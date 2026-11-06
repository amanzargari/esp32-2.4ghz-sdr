#include <stddef.h>
#include <stdint.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "link.h"
#include "protocol.h"
#include "rf_capture.h"
#include "stream.h"

#define LINK_TASK_CORE     0
#define LINK_TASK_STACK    4096
#define LINK_TASK_PRIORITY 5

static uint32_t s_heartbeat_counter;

static void send_heartbeat(void)
{
    heartbeat_t packet = {
        .magic = HEARTBEAT_MAGIC,
        .timestamp_us = (uint64_t)esp_timer_get_time(),
        .counter = s_heartbeat_counter++,
        .rx_errors = link_rx_errors(),
    };
    const uint8_t *bytes = (const uint8_t *)&packet;
    uint8_t checksum = 0;

    for (size_t i = 0; i < sizeof(packet) - 1; i++) {
        checksum ^= bytes[i];
    }
    packet.checksum = checksum;
    link_send(&packet, sizeof(packet));
}

static void on_link_rx(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (data[i] == CMD_PING) {
            send_heartbeat();
        }
    }
}

static void on_link_idle(void)
{
}

static void link_task(void *arg)
{
    link_run(on_link_rx, on_link_idle);
}

void app_main(void)
{
    link_init();
    rf_capture_init();
    stream_start();
    xTaskCreatePinnedToCore(link_task, "link", LINK_TASK_STACK, NULL,
                            LINK_TASK_PRIORITY, NULL, LINK_TASK_CORE);
}
