#include "spi_link.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "protocol.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

// These are VSPI's native IO_MUX pins, which bypass the slower GPIO matrix.
#define SPI_LINK_HOST       SPI3_HOST
#define SPI_LINK_SCLK_GPIO  18
#define SPI_LINK_MISO_GPIO  19
#define SPI_LINK_MOSI_GPIO  23
#define SPI_LINK_CS_GPIO    5
#define SPI_LINK_READY_GPIO 22

// With DMA, modes 0 and 2 launch MISO half a clock early. ESP-IDF recommends modes 1 and 3
// instead, but with the Pi 5 as master those failed from 10 MHz up, and mode 0 ran cleanly
// at 8-12.5 MHz and at 20-25 MHz (where pi_receiver.py compensates for a one-bit delay).
#define SPI_LINK_MODE       0

// Activity on CS just after a transfer ends the next frame if it is armed by then. The
// caller resends such frames, and waiting this long first makes that rare.
#define SPI_LINK_REARM_DELAY_US 200

static void IRAM_ATTR on_frame_armed(spi_slave_transaction_t *trans)
{
    if (trans->user) {
        WRITE_PERI_REG(GPIO_OUT_W1TS_REG, BIT(SPI_LINK_READY_GPIO));
    }
}

static void IRAM_ATTR on_frame_done(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, BIT(SPI_LINK_READY_GPIO));
}

void spi_link_init(void)
{
    const gpio_config_t ready = {
        .pin_bit_mask = BIT64(SPI_LINK_READY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    const spi_bus_config_t bus = {
        .mosi_io_num = SPI_LINK_MOSI_GPIO,
        .miso_io_num = SPI_LINK_MISO_GPIO,
        .sclk_io_num = SPI_LINK_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_FRAME_SIZE,
    };
    const spi_slave_interface_config_t slave = {
        .spics_io_num = SPI_LINK_CS_GPIO,
        .mode = SPI_LINK_MODE,
        .queue_size = 1,
        .post_setup_cb = on_frame_armed,
        .post_trans_cb = on_frame_done,
    };

    ESP_ERROR_CHECK(gpio_config(&ready));
    ESP_ERROR_CHECK(gpio_set_level(SPI_LINK_READY_GPIO, 0));
    // Keeps the bus from clocking noise while the Pi's SPI pins are unconfigured.
    ESP_ERROR_CHECK(gpio_set_pull_mode(SPI_LINK_SCLK_GPIO, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SPI_LINK_CS_GPIO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SPI_LINK_MOSI_GPIO, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_LINK_HOST, &bus, &slave, SPI_DMA_CH_AUTO));
    // MISO's timing margin is what limits the clock, and jumper wires load it.
    ESP_ERROR_CHECK(gpio_set_drive_capability(SPI_LINK_MISO_GPIO, GPIO_DRIVE_CAP_3));
}

bool spi_link_exchange(const void *tx, void *rx, bool signal)
{
    spi_slave_transaction_t trans = {
        .length = SPI_FRAME_SIZE * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .user = signal ? &trans : NULL,
    };
    spi_slave_transaction_t *done;

    esp_rom_delay_us(SPI_LINK_REARM_DELAY_US);
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_LINK_HOST, &trans, portMAX_DELAY));
    ESP_ERROR_CHECK(spi_slave_get_trans_result(SPI_LINK_HOST, &done, portMAX_DELAY));
    return done->trans_len >= SPI_FRAME_SIZE * 8;
}
