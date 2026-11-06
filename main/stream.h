// Captures bursts and streams them to the SPI master through two DMA burst buffers.
#pragma once

// Starts the capture and SPI tasks. rf_capture_init() must have run first.
void stream_start(void);
