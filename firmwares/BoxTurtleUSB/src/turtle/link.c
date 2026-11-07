// USB line link for the BoxTurtle firmware.
//
// Bytes from the host are cut into lines at '\n' and handed to
// proto_handle_line() from task context. Output lines go into a ring
// that the USB bulk-in task drains.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "generic/usb_cdc.h" // usb_notify_bulk_in
#include "sched.h" // DECL_TASK
#include "board.h" // uptime
#include "link.h"

#define LINE_MAX 256
#define TX_SIZE 4096

static char rx_line[LINE_MAX];
static int rx_len, rx_overflow;
static uint32_t last_rx;
static int host_seen;
static uint32_t rx_lines, tx_lines;
// The first bytes of the last few lines, for the rxlog bench command.
#define RXLOG_N 6
#define RXLOG_BYTES 20
static uint8_t rxlog[RXLOG_N][RXLOG_BYTES], rxlog_len[RXLOG_N], rxlog_pos;

int
turtle_link_rx(const uint8_t *data, int len)
{
    int i;
    if (len > 0)
        last_rx = uptime();
    for (i = 0; i < len; i++) {
        char c = data[i];
        // '\r' ends a line too, so a modem probe ("AT\r") from ModemManager
        // never runs into the next JSON line.
        if (c == '\n' || c == '\r') {
            if (!rx_overflow && rx_len) {
                rx_line[rx_len] = 0;
                host_seen = 1;
                rx_lines++;
                int n = rx_len < RXLOG_BYTES ? rx_len : RXLOG_BYTES;
                memcpy(rxlog[rxlog_pos], rx_line, n);
                rxlog_len[rxlog_pos] = n;
                rxlog_pos = (rxlog_pos + 1) % RXLOG_N;
                proto_handle_line(rx_line);
            }
            rx_len = 0;
            rx_overflow = 0;
            continue;
        }
        if (rx_len >= LINE_MAX - 1) {
            rx_overflow = 1;
            continue;
        }
        rx_line[rx_len++] = c;
    }
    return len;
}

uint32_t
link_last_rx(void)
{
    return last_rx;
}

int
link_host_seen(void)
{
    return host_seen;
}

/****************************************************************
 * Output ring
 ****************************************************************/

static uint8_t tx_ring[TX_SIZE];
static uint32_t tx_head, tx_tail; // head = write, tail = read (free-running)
static uint32_t tx_dropped;

int
link_send(const char *line, int len)
{
    if (len <= 0)
        return 0;
    if (TX_SIZE - (tx_head - tx_tail) < (uint32_t)len) {
        tx_dropped++;
        return 0;
    }
    int i;
    for (i = 0; i < len; i++)
        tx_ring[(tx_head + i) % TX_SIZE] = line[i];
    tx_head += len;
    tx_lines++;
    usb_notify_bulk_in();
    return 1;
}

int
turtle_link_tx_fill(uint8_t *dst, int space)
{
    int n = 0;
    while (n < space && tx_tail != tx_head) {
        dst[n++] = tx_ring[tx_tail % TX_SIZE];
        tx_tail++;
    }
    return n;
}

uint32_t
link_dropped(void)
{
    return tx_dropped;
}

// The last lines' first bytes as a JSON array of hex strings, oldest first.
void
link_rxlog(char *out, int size)
{
    static const char hex[] = "0123456789abcdef";
    int o = 0, i, j;
    out[o++] = '[';
    for (i = 0; i < RXLOG_N; i++) {
        int k = (rxlog_pos + i) % RXLOG_N;
        if (!rxlog_len[k])
            continue;
        if (o + 2 * RXLOG_BYTES + 4 >= size)
            break;
        if (o > 1)
            out[o++] = ',';
        out[o++] = '"';
        for (j = 0; j < rxlog_len[k]; j++) {
            out[o++] = hex[rxlog[k][j] >> 4];
            out[o++] = hex[rxlog[k][j] & 15];
        }
        out[o++] = '"';
    }
    out[o++] = ']';
    out[o] = 0;
}

uint32_t
link_rx_lines(void)
{
    return rx_lines;
}

uint32_t
link_tx_lines(void)
{
    return tx_lines;
}
