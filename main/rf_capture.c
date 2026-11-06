// The dump registers are undocumented. Their addresses and the capture sequence
// come from reverse-engineering work on adctrig() in Espressif's librftest.a, as
// published by the ESP-SDR project (github.com/ESPARGOS/esp-sdr), and were checked
// against a disassembly of that function. They, and the private PHY functions
// below, are tied to the ESP-IDF version: re-verify after upgrading from v5.5.4.
#include "rf_capture.h"

#include <stdbool.h>

#include "esp_phy_init.h"
#include "esp_private/periph_ctrl.h"
#include "esp_timer.h"
#include "heap_memory_layout.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "soc/soc.h"

#define DUMP_CTRL_REG      0x60033d90u
#define DUMP_STATUS_REG    0x60033d94u
#define DUMP_BYTES_REG     0x60033dc4u
#define RX_GAIN_REG        0x3ff5c02cu

#define DUMP_CTRL_ENABLE   BIT(31)
#define DUMP_CTRL_TRIGGER  BIT(19)
#define DUMP_CTRL_DONE     BIT(18)
#define DUMP_CTRL_CLK_16M  BIT(16)
#define DUMP_CTRL_CLK_40M  BIT(15)
#define DUMP_STATUS_COUNT  0x7fffu
#define DUMP_BYTES_IN_ORDER 0x03020100u
#define RX_GAIN_FORCE      BIT(23)
#define RX_GAIN_MAX_S      8
#define RX_GAIN_MAX_V      0x7fu
#define RX_GAIN_FORCED_S   24
#define RX_GAIN_FORCED_M   (0xffu << RX_GAIN_FORCED_S)

// Expires the cached DC and gain tables but keeps the startup IQ correction (bit
// 0x400), so the rebuild doesn't rerun the IQ calibration that transmits a tone.
#define PHY_CAL_DC_STALE   0x20320u

#define PLL_BASE_MHZ       2412
#define PLL_I2C_BLOCK      0x62
#define PLL_I2C_HOST       1
#define PLL_I2C_CAP_REG    0
#define PLL_CAP_PINNED     0x80u
#define ROM_CRYSTAL_40MHZ  0 // The ROM's code for this board's crystal

#define DUMP_MODE_FULL     3 // Gives the engine the whole 64 KiB aperture
#define DUMP_START         0x3ffe8000u
#define DUMP_END           0x3fff8000u
#define DUMP_WORDS         16384u
#define DUMP_SENTINEL      0xa55aa55au
#define DUMP_TIMEOUT_US    20000

// While capturing, this SRAM belongs to the modem, so the CPU must not use it.
SOC_RESERVE_MEMORY_REGION(DUMP_START, DUMP_END, rf_dump);

static uint32_t *const s_dump = (uint32_t *)DUMP_START;

// Private PHY functions with no public header.
extern uint32_t chip7_sleep_params[];
extern void set_chanfreq(unsigned mhz, unsigned mode);
extern void set_rx_gain_table(unsigned mhz, unsigned mode);
extern unsigned rom_chip_i2c_readReg(unsigned block, unsigned host, unsigned reg);
extern void rom_chip_i2c_writeReg(unsigned block, unsigned host, unsigned reg, unsigned value);
extern void rom_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_pbus_xpd_rx_on(int enable);
extern void rom_set_rxclk_en(int enable);

void rf_capture_init(void)
{
    // The modem registers read as zero until this domain is on, whatever the clocks say.
    esp_wifi_bt_power_domain_on();
    wifi_bt_common_module_enable();
    wifi_module_enable();
    esp_phy_enable(PHY_MODEM_WIFI);
}

static bool is_wifi_channel(unsigned mhz)
{
    return mhz == 2484 || (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0);
}

// The PHY only tunes to Wi-Fi channels, so anything else starts from a channel and then
// retunes with the ROM. Channel setup pins the PLL's capacitor code, which has to be
// released first or the synthesizer changes while the PLL stays unlocked.
static void tune_pll(unsigned mhz)
{
    if (is_wifi_channel(mhz)) {
        set_chanfreq(mhz, 0);
        return;
    }

    set_chanfreq(PLL_BASE_MHZ, 0);
    unsigned cap = rom_chip_i2c_readReg(PLL_I2C_BLOCK, PLL_I2C_HOST, PLL_I2C_CAP_REG);
    rom_chip_i2c_writeReg(PLL_I2C_BLOCK, PLL_I2C_HOST, PLL_I2C_CAP_REG, cap & ~PLL_CAP_PINNED);
    rom_set_rf_freq_offset(ROM_CRYSTAL_40MHZ, mhz, 0);
}

esp_err_t rf_capture_tune(uint16_t mhz)
{
    if (mhz < RF_TUNE_MIN_MHZ || mhz > RF_TUNE_MAX_MHZ) {
        return ESP_ERR_INVALID_ARG;
    }

    REG_CLR_BIT(RX_GAIN_REG, RX_GAIN_FORCE); // A forced gain would defeat the DC sweep
    tune_pll(mhz);
    chip7_sleep_params[0] &= ~PHY_CAL_DC_STALE;
    set_rx_gain_table(mhz, 0);
    tune_pll(mhz);

    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);

    // The first capture after a retune starts with about 7 us of excess power, however
    // long we wait first. A short capture clears it, so that one is discarded here.
    const uint32_t *discarded;
    uint32_t elapsed_us;
    return rf_capture_acquire(RF_RATE_80_MSPS, RF_CAPTURE_MIN_SAMPLES, &discarded, &elapsed_us);
}

esp_err_t rf_capture_set_gain(uint8_t gain)
{
    uint32_t reg = REG_READ(RX_GAIN_REG);

    if (gain == RF_GAIN_AGC) {
        REG_WRITE(RX_GAIN_REG, reg & ~RX_GAIN_FORCE);
        return ESP_OK;
    }
    if (gain > ((reg >> RX_GAIN_MAX_S) & RX_GAIN_MAX_V)) {
        return ESP_ERR_INVALID_ARG;
    }
    REG_WRITE(RX_GAIN_REG, (reg & ~RX_GAIN_FORCED_M) | ((uint32_t)gain << RX_GAIN_FORCED_S) |
                               RX_GAIN_FORCE);
    return ESP_OK;
}

static uint32_t clock_bits(rf_rate_t rate)
{
    switch (rate) {
    case RF_RATE_40_MSPS:
        return DUMP_CTRL_CLK_40M;
    case RF_RATE_16_MSPS:
        return DUMP_CTRL_CLK_16M;
    default:
        return 0;
    }
}

esp_err_t rf_capture_acquire(rf_rate_t rate, size_t samples, const uint32_t **data,
                             uint32_t *elapsed_us)
{
    if (samples < RF_CAPTURE_MIN_SAMPLES || samples > RF_CAPTURE_MAX_SAMPLES ||
        rate > RF_RATE_16_MSPS) {
        return ESP_ERR_INVALID_ARG;
    }

    REG_WRITE(DUMP_CTRL_REG, 0);
    for (size_t i = 0; i < DUMP_WORDS; i++) {
        s_dump[i] = DUMP_SENTINEL;
    }

    uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
    uint32_t bytes = REG_READ(DUMP_BYTES_REG);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
                    (owner & ~DPORT_MAC_DUMP_MODE_M) | (DUMP_MODE_FULL << DPORT_MAC_DUMP_MODE_S));
    REG_WRITE(DUMP_BYTES_REG, DUMP_BYTES_IN_ORDER);

    REG_WRITE(DUMP_CTRL_REG, samples | clock_bits(rate) | DUMP_CTRL_ENABLE);
    int64_t start = esp_timer_get_time();
    REG_SET_BIT(DUMP_CTRL_REG, DUMP_CTRL_TRIGGER);
    REG_CLR_BIT(DUMP_CTRL_REG, DUMP_CTRL_TRIGGER);
    while (!(REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) &&
           esp_timer_get_time() - start < DUMP_TIMEOUT_US) {
    }
    uint32_t ctrl = REG_READ(DUMP_CTRL_REG);
    uint32_t status = REG_READ(DUMP_STATUS_REG);
    *elapsed_us = (uint32_t)(esp_timer_get_time() - start);

    REG_WRITE(DUMP_CTRL_REG, 0);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
    REG_WRITE(DUMP_BYTES_REG, bytes);

    if (!(ctrl & DUMP_CTRL_DONE)) {
        return ESP_ERR_TIMEOUT;
    }
    if ((status & DUMP_STATUS_COUNT) != samples) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (size_t i = 0; i < DUMP_WORDS; i++) {
        if ((s_dump[i] == DUMP_SENTINEL) == (i < samples)) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    *data = s_dump;
    return ESP_OK;
}
