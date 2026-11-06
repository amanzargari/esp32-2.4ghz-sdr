// The radio fills one burst buffer while the SPI master drains the other. The dump
// engine only takes snapshots, so bursts are separated by gaps: a burst takes the radio
// at most 1 ms to capture but tens of milliseconds to send.
#include "stream.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "protocol.h"
#include "rf_capture.h"
#include "spi_link.h"

#define BURST_BUFFERS       2
#define BURST_MAX_FRAMES    ((RF_CAPTURE_MAX_SAMPLES + SPI_FRAME_SAMPLES - 1) / SPI_FRAME_SAMPLES)
#define NO_BUFFER           (-1)

// The capture task shares core 0 with the UART link and the PHY's startup calibration.
#define CAPTURE_TASK_CORE   0
#define CAPTURE_TASK_STACK  4096
#define CAPTURE_TASK_PRIORITY 6
#define SPI_TASK_CORE       1
#define SPI_TASK_STACK      4096
#define SPI_TASK_PRIORITY   10

typedef struct {
    const sdr_frame_t *frames;
    size_t count;
    int buffer;
} stream_item_t;

static WORD_ALIGNED_ATTR sdr_frame_t s_buffers[BURST_BUFFERS][BURST_MAX_FRAMES];
static WORD_ALIGNED_ATTR sdr_frame_t s_idle_frame;
static WORD_ALIGNED_ATTR sdr_frame_t s_status_frame;
static WORD_ALIGNED_ATTR sdr_frame_t s_end_frame;
static WORD_ALIGNED_ATTR sdr_frame_t s_rx_frame;

static QueueHandle_t s_free_buffers; // Buffers the radio can fill
static QueueHandle_t s_tx_items;     // Filled buffers and the end frame, in order
static TaskHandle_t s_capture_task;

static sdr_command_t s_request;
static volatile bool s_stop;
static uint32_t s_sequence; // Written only by the capture task, while streaming
static uint16_t s_tuned_mhz;

static capture_status_t to_capture_status(esp_err_t err)
{
    switch (err) {
    case ESP_OK:
        return CAPTURE_OK;
    case ESP_ERR_INVALID_ARG:
        return CAPTURE_ERR_ARGS;
    case ESP_ERR_TIMEOUT:
        return CAPTURE_ERR_TIMEOUT;
    default:
        return CAPTURE_ERR_MEMORY;
    }
}

static void seal_frame(sdr_frame_t *frame)
{
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)frame, offsetof(sdr_frame_t, crc32));

    frame->crc32 = esp_rom_crc32_le(crc, frame->payload, sizeof(frame->payload));
}

static void write_control_frame(sdr_frame_t *frame, uint8_t type, capture_status_t status,
                                uint32_t sequence, uint32_t burst)
{
    memset(frame, 0, sizeof(*frame));
    frame->magic = SPI_FRAME_MAGIC;
    frame->type = type;
    frame->status = status;
    frame->sequence = sequence;
    frame->burst = burst;
    frame->timestamp_us = (uint32_t)esp_timer_get_time();
    seal_frame(frame);
}

static uint16_t channel_to_mhz(uint8_t channel)
{
    return channel == 14 ? 2484 : 2407 + 5 * channel;
}

static capture_status_t capture_burst(uint8_t channel, uint32_t **data)
{
    esp_err_t err = ESP_OK;
    uint32_t elapsed_us;
    uint16_t mhz = s_request.frequency_mhz ? s_request.frequency_mhz : channel_to_mhz(channel);

    if (mhz != s_tuned_mhz) {
        err = rf_capture_tune(mhz);
        s_tuned_mhz = err == ESP_OK ? mhz : 0;
    }
    if (err == ESP_OK) {
        err = rf_capture_set_gain(s_request.gain);
    }
    if (err == ESP_OK) {
        err = rf_capture_acquire(s_request.rate, s_request.samples, (const uint32_t **)data,
                                 &elapsed_us);
    }
    return to_capture_status(err);
}

static size_t fill_burst_frames(sdr_frame_t *frames, const uint32_t *data, uint8_t channel,
                                uint32_t burst)
{
    uint32_t timestamp = (uint32_t)esp_timer_get_time();
    size_t count = 0;

    for (size_t offset = 0; offset < s_request.samples; offset += SPI_FRAME_SAMPLES) {
        sdr_frame_t *frame = &frames[count++];
        size_t samples = MIN(s_request.samples - offset, SPI_FRAME_SAMPLES);

        memset(frame, 0, sizeof(*frame));
        frame->magic = SPI_FRAME_MAGIC;
        frame->type = SPI_FRAME_DATA;
        frame->channel = channel;
        frame->rate = s_request.rate;
        frame->gain = s_request.gain;
        frame->sequence = s_sequence++;
        frame->burst = burst;
        frame->sample_offset = offset;
        frame->sample_count = samples;
        frame->burst_samples = s_request.samples;
        frame->timestamp_us = timestamp;
        memcpy(frame->payload, &data[offset], samples * sizeof(uint32_t));
        seal_frame(frame);
    }
    return count;
}

static void capture_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        capture_status_t status = CAPTURE_OK;
        uint32_t channels = s_request.frequency_mhz
                                ? 1
                                : s_request.last_channel - s_request.first_channel + 1;
        uint32_t total_bursts = s_request.passes * channels;
        uint32_t burst = 0;

        while (status == CAPTURE_OK && !s_stop &&
               (s_request.passes == 0 || burst < total_bursts)) {
            int buffer;
            xQueueReceive(s_free_buffers, &buffer, portMAX_DELAY);

            uint8_t channel = s_request.frequency_mhz
                                  ? 0
                                  : s_request.first_channel + burst % channels;
            uint32_t *data;
            status = capture_burst(channel, &data);
            if (status != CAPTURE_OK) {
                xQueueSend(s_free_buffers, &buffer, portMAX_DELAY);
                break;
            }

            stream_item_t item = {
                .frames = s_buffers[buffer],
                .count = fill_burst_frames(s_buffers[buffer], data, channel, burst),
                .buffer = buffer,
            };
            xQueueSend(s_tx_items, &item, portMAX_DELAY);
            burst++;
        }

        write_control_frame(&s_end_frame, SPI_FRAME_END, status, s_sequence++, burst);
        const stream_item_t end = {.frames = &s_end_frame, .count = 1, .buffer = NO_BUFFER};
        xQueueSend(s_tx_items, &end, portMAX_DELAY);
    }
}

static bool read_command(sdr_command_t *command)
{
    memcpy(command, &s_rx_frame, sizeof(*command));
    return command->magic == SPI_COMMAND_MAGIC &&
           command->crc32 == esp_rom_crc32_le(0, (const uint8_t *)command,
                                              offsetof(sdr_command_t, crc32));
}

static void spi_task(void *arg)
{
    stream_item_t current = {.buffer = NO_BUFFER};
    bool streaming = false;
    bool status_pending = false;
    const sdr_frame_t *undelivered = NULL;

    spi_link_init();
    write_control_frame(&s_idle_frame, SPI_FRAME_IDLE, CAPTURE_OK, 0, 0);

    for (;;) {
        const sdr_frame_t *frame;
        bool signal = true;

        if (undelivered) {
            frame = undelivered;
        } else if (status_pending) {
            frame = &s_status_frame;
            status_pending = false;
        } else {
            if (current.count == 0 && streaming) {
                xQueueReceive(s_tx_items, &current, portMAX_DELAY);
            }
            if (current.count > 0) {
                frame = current.frames++;
                current.count--;
            } else {
                // Only armed while no capture is running, so data can't queue behind it.
                frame = &s_idle_frame;
                signal = false;
            }
        }

        bool delivered = spi_link_exchange(frame, &s_rx_frame, signal);
        undelivered = delivered || frame == &s_idle_frame ? NULL : frame;
        if (undelivered) {
            continue;
        }

        if (frame == &s_end_frame) {
            streaming = false;
        }
        if (current.count == 0 && current.buffer != NO_BUFFER) {
            xQueueSend(s_free_buffers, &current.buffer, portMAX_DELAY);
            current.buffer = NO_BUFFER;
        }

        sdr_command_t command;
        if (!read_command(&command)) {
            continue;
        }
        if (command.command == SPI_COMMAND_STOP) {
            s_stop = true;
            if (!streaming) {
                write_control_frame(&s_status_frame, SPI_FRAME_STATUS, CAPTURE_OK, 0, 0);
                status_pending = true;
            }
        } else if (command.command == SPI_COMMAND_START) {
            capture_status_t status = streaming ? CAPTURE_ERR_BUSY : CAPTURE_OK;
            if (!command.frequency_mhz &&
                (command.first_channel < 1 || command.last_channel > 14 ||
                 command.last_channel < command.first_channel)) {
                status = CAPTURE_ERR_ARGS;
            }
            write_control_frame(&s_status_frame, SPI_FRAME_STATUS, status, 0, 0);
            status_pending = true;
            if (status == CAPTURE_OK) {
                s_request = command;
                s_stop = false;
                streaming = true;
                xTaskNotifyGive(s_capture_task);
            }
        }
    }
}

void stream_start(void)
{
    s_free_buffers = xQueueCreate(BURST_BUFFERS, sizeof(int));
    s_tx_items = xQueueCreate(BURST_BUFFERS + 1, sizeof(stream_item_t));
    configASSERT(s_free_buffers && s_tx_items);

    for (int buffer = 0; buffer < BURST_BUFFERS; buffer++) {
        xQueueSend(s_free_buffers, &buffer, 0);
    }

    xTaskCreatePinnedToCore(capture_task, "capture", CAPTURE_TASK_STACK, NULL,
                            CAPTURE_TASK_PRIORITY, &s_capture_task, CAPTURE_TASK_CORE);
    xTaskCreatePinnedToCore(spi_task, "spi", SPI_TASK_STACK, NULL,
                            SPI_TASK_PRIORITY, NULL, SPI_TASK_CORE);
}
