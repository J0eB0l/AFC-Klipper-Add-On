// USB line link for the BoxTurtle firmware.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_LINK_H
#define __TURTLE_LINK_H

#include <stdint.h>

// Called by the patched generic/usb_cdc.c.
// Consumes received bytes; returns how many were taken.
int turtle_link_rx(const uint8_t *data, int len);
// Copies queued output into dst; returns how many bytes.
int turtle_link_tx_fill(uint8_t *dst, int space);

// Queues one whole line for the host. Lines are dropped whole, never cut,
// when the queue is full. Returns 1 if queued.
int link_send(const char *line, int len);
// uptime() in ms of the last byte from the host.
uint32_t link_last_rx(void);
// 1 once the host has sent at least one line since power up.
int link_host_seen(void);
uint32_t link_dropped(void);
// Lines received (non-empty) and lines queued for sending, since boot.
uint32_t link_rx_lines(void);
uint32_t link_tx_lines(void);
// Bench: the last lines' first bytes, as a JSON array of hex strings.
void link_rxlog(char *out, int size);

// Implemented in proto.c: handles one received line (no newline).
void proto_handle_line(char *line);

#endif // link.h
