#include "link.h"

#include <sys/param.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/uart_ll.h"

#define LINK_UART            UART_NUM_0
#define LINK_TX_GPIO         1
#define LINK_RX_GPIO         3
#define LINK_RX_BUF_SIZE     1024
#define LINK_TX_BUF_SIZE     4096
#define LINK_EVENT_QUEUE_LEN 16
#define LINK_IDLE_TICKS      pdMS_TO_TICKS(100)

static QueueHandle_t s_uart_events;
static uint16_t s_rx_errors;

void link_init(void)
{
    const uart_config_t config = {
        .baud_rate = CONFIG_SDR_LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, LINK_RX_BUF_SIZE, LINK_TX_BUF_SIZE,
                                        LINK_EVENT_QUEUE_LEN, &s_uart_events, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_TX_GPIO, LINK_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // The driver leaves this off by default, so UART_FRAME_ERR would never fire.
    ESP_ERROR_CHECK(uart_enable_intr_mask(LINK_UART, UART_INTR_FRAM_ERR));
}

void link_send(const void *data, size_t len)
{
    uart_write_bytes(LINK_UART, data, len);
}

uint16_t link_rx_errors(void)
{
    return s_rx_errors;
}

static void read_pending(size_t len, link_rx_handler_t on_rx)
{
    uint8_t buf[64];

    while (len > 0) {
        int n = uart_read_bytes(LINK_UART, buf, MIN(len, sizeof(buf)), 0);
        if (n <= 0) {
            return;
        }
        on_rx(buf, n);
        len -= n;
    }
}

void link_run(link_rx_handler_t on_rx, link_idle_handler_t on_idle)
{
    uart_event_t event;

    for (;;) {
        if (xQueueReceive(s_uart_events, &event, LINK_IDLE_TICKS) != pdTRUE) {
            on_idle();
            continue;
        }

        switch (event.type) {
        case UART_DATA:
            read_pending(event.size, on_rx);
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            s_rx_errors++;
            uart_flush_input(LINK_UART);
            xQueueReset(s_uart_events);
            break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BREAK:
            s_rx_errors++;
            break;
        default:
            break;
        }
    }
}
