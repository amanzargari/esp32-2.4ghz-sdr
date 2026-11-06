// SPI slave link to the Raspberry Pi: fixed-size DMA frames paced by a handshake line.
#pragma once

#include <stdbool.h>

// Must run on the core that should service the SPI interrupt.
void spi_link_init(void);

// Arms one full-duplex frame of SPI_FRAME_SIZE bytes and blocks until the master has
// clocked it. Both buffers must be DMA capable and word aligned. If `signal` is set,
// the handshake line is high from when the frame is armed until the master finishes.
// Returns whether the master clocked the whole frame. If not, it was never delivered.
bool spi_link_exchange(const void *tx, void *rx, bool signal);
