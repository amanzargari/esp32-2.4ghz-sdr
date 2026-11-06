// Wire formats shared with tools/test_link.py (UART) and tools/pi_receiver.py (SPI).
// All fields are little-endian.
#pragma once

#include <stddef.h>
#include <stdint.h>

// --- UART: link health only. Capture payloads travel over SPI.

#define CMD_PING             0xAA
#define HEARTBEAT_MAGIC      0x55

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint64_t timestamp_us;
    uint32_t counter;
    uint16_t rx_errors; // UART receive errors seen by the ESP32 since boot
    uint8_t checksum;   // XOR of the preceding bytes
} heartbeat_t;

_Static_assert(sizeof(heartbeat_t) == 16, "heartbeat must be 16 bytes");

// --- SPI: every transaction is one full-duplex frame of SPI_FRAME_SIZE bytes. The Pi
// sends a sdr_command_t, or zeros, while it reads a sdr_frame_t.

#define SPI_FRAME_SIZE       4096
#define SPI_FRAME_HEADER     32
#define SPI_FRAME_SAMPLES    ((SPI_FRAME_SIZE - SPI_FRAME_HEADER) / 4)

#define SPI_FRAME_MAGIC      0x46524453 // "SDRF" on the wire
#define SPI_COMMAND_MAGIC    0x43524453 // "SDRC" on the wire

#define CAPTURE_GAIN_AGC     0xFF

typedef enum {
    SPI_FRAME_IDLE = 0,   // Armed with the handshake low, so the Pi can send a command
    SPI_FRAME_STATUS = 1, // Reply to a command
    SPI_FRAME_DATA = 2,   // Part of a burst
    SPI_FRAME_END = 3,    // Stream finished. `burst` is the number captured
} sdr_frame_type_t;

typedef enum {
    CAPTURE_OK = 0,
    CAPTURE_ERR_ARGS = 1,
    CAPTURE_ERR_TIMEOUT = 2,
    CAPTURE_ERR_MEMORY = 3,
    CAPTURE_ERR_BUSY = 4,
} capture_status_t;

// Samples are raw words: bits 19:10 I, 9:0 Q, 26:20 gain index.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t type;           // sdr_frame_type_t
    uint8_t channel;        // 0 if tuned by frequency
    uint8_t rate;           // rf_rate_t
    uint8_t gain;           // As requested: index or CAPTURE_GAIN_AGC
    uint32_t sequence;      // Increments on every DATA and END frame
    uint32_t burst;
    uint16_t sample_offset; // Position of this payload within the burst
    uint16_t sample_count;  // Valid samples in this payload
    uint16_t burst_samples;
    uint8_t status;         // capture_status_t
    uint8_t reserved;
    uint32_t timestamp_us;  // Low 32 bits of the ESP32 clock
    uint32_t crc32;         // zlib CRC-32 of the rest of the frame, skipping this field
    uint8_t payload[SPI_FRAME_SIZE - SPI_FRAME_HEADER];
} sdr_frame_t;

_Static_assert(sizeof(sdr_frame_t) == SPI_FRAME_SIZE, "SPI frame size");
_Static_assert(offsetof(sdr_frame_t, payload) == SPI_FRAME_HEADER, "SPI frame header size");

typedef enum {
    SPI_COMMAND_START = 1,
    SPI_COMMAND_STOP = 2,
} sdr_command_type_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t command;  // sdr_command_type_t
    uint8_t first_channel; // Wi-Fi channel, 1-14
    uint8_t last_channel;  // Same as first_channel to stay on one channel
    uint8_t rate;     // rf_rate_t
    uint8_t gain;     // RX gain table index, or CAPTURE_GAIN_AGC for hardware AGC
    uint16_t samples; // Per burst
    uint16_t passes;  // Passes through the channel range. 0 means until SPI_COMMAND_STOP
    uint16_t frequency_mhz; // If not 0, stay on this frequency and ignore the channels
    uint32_t crc32;   // zlib CRC-32 of the preceding bytes
} sdr_command_t;
