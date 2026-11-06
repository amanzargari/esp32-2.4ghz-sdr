// Binary UART link to the PC.
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void (*link_rx_handler_t)(const uint8_t *data, size_t len);
typedef void (*link_idle_handler_t)(void);

void link_init(void);

// All outbound data goes through here. Blocks until `len` bytes are queued.
void link_send(const void *data, size_t len);

// UART receive errors (framing, parity, break, overflow) since boot.
uint16_t link_rx_errors(void);

// Delivers received bytes to `on_rx`, and calls `on_idle` after a quiet period.
// Never returns.
void link_run(link_rx_handler_t on_rx, link_idle_handler_t on_idle);
