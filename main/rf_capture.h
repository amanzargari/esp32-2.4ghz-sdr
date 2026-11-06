// Receive-only raw I/Q capture through the ESP32's undocumented modem dump engine.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define RF_CAPTURE_MIN_SAMPLES 256
#define RF_CAPTURE_MAX_SAMPLES 16380
#define RF_GAIN_AGC            0xFF
#define RF_TUNE_MIN_MHZ        2400
#define RF_TUNE_MAX_MHZ        2500

typedef enum {
    RF_RATE_80_MSPS = 0,
    RF_RATE_40_MSPS = 1,
    RF_RATE_16_MSPS = 2,
} rf_rate_t;

// Powers on and calibrates the PHY. The Wi-Fi driver and MAC are never started.
void rf_capture_init(void);

// Locks the PLL to a frequency in receive mode and remeasures the RX DC offset there,
// which takes about 8 ms. This releases any forced gain. Can also return the errors of
// rf_capture_acquire().
esp_err_t rf_capture_tune(uint16_t mhz);

// Forces the RX gain table index, which is not a value in dB, or restores the
// hardware AGC with RF_GAIN_AGC. The highest index comes from the PHY's gain table,
// which rf_capture_tune() rebuilds, so call this after tuning.
esp_err_t rf_capture_set_gain(uint8_t gain);

// Captures raw words: bits 19:10 I, 9:0 Q, 26:20 gain index. `*data` stays valid
// until the next capture.
// Returns ESP_ERR_INVALID_ARG, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_RESPONSE if the
// engine did not write exactly `samples` words.
esp_err_t rf_capture_acquire(rf_rate_t rate, size_t samples, const uint32_t **data,
                             uint32_t *elapsed_us);
