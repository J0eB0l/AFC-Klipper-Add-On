// BridgeHub: the ESP32-S3's own USB port as a USB host for one serial unit.
//
// A self-powered unit (a USB BoxTurtle, an ACE) plugs into the S3's own USB
// port. The hub opens it as a CDC-ACM serial device and serves its bytes raw
// on TCP port HUB_TCP_PORT, the way ser2net does: the host connects to that
// port and talks to the unit as if it were plugged in.
//
// RAW TCP CARRIES NO BAUD RATE. The hub opens each unit at its own speed:
// an ACE (WCH 1a86) at 230400, anything else at 115200. A USB BoxTurtle is
// USB CDC and ignores the speed, but never 1200, which asks it for its
// bootloader.
//
// No 5 V goes out of the port: a self-powered unit doesn't need it (the
// Turtle's USB never checks VBUS). A USB hub, or a unit whose USB chip runs
// off VBUS, needs 5 V passed to the port.
//
// THE LINK KEY. A hub answers every new connection with a greeting line
// before any unit bytes:
//   keyed: {"evt":"auth","nonce":"<32 hex>"}
//          the client answers {"cmd":"auth","mac":"<64 hex>"}, the first 32
//          bytes of HMAC-SHA512(key, nonce), and gets {"evt":"auth","ok":1}
//          (or {"evt":"auth","ok":0} and the connection closed)
//   open:  {"evt":"auth","open":1}
//          a client that sends {"cmd":"linkkey","key":"<key>"} as its first
//          line claims the hub: the key is saved and asked for from then on
// A keyed hub only lets a connection that proved the key take over from
// the current client, so nothing else on the network can push AFC off. An
// open hub hands over to any new connection. Holding BOOT for 10 s forgets
// the key along with the network (wifi.c).
//
// Everything here runs on core 0, with the WiFi driver.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "esp_intr_alloc.h"
#include "esp_random.h"
#include "nvs.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "lwip/sockets.h"

#include "hub.h"
#include "linkkey.h"

#define HUB_TCP_PORT     4001
#define HUB_CORE         0
#define HUB_RX_BUF       8192      // unit -> TCP, while the client catches up
#define HUB_XFER_SIZE    512
#define HUB_OPEN_WAIT_MS 1000
#define HUB_TX_WAIT_MS   500
#define HUB_VID_WCH      0x1A86    // the ACE's USB serial chip
#define HUB_BAUD_ACE     230400
#define HUB_BAUD_OTHER   115200
#define HUB_KEYED_MS     5000      // a keyed hub's wait for the answer
#define HUB_OPEN_MS      2000      // an open hub's wait for a claim
#define HUB_NVS_NS       "bridgehub"
#define HUB_NVS_KEY      "linkkey"

static StreamBufferHandle_t s_rx;            // unit -> TCP
static SemaphoreHandle_t s_sock_mux;         // guards s_client
static volatile int s_client = -1;
static volatile cdc_acm_dev_hdl_t s_dev;
static volatile bool s_gone;                 // set by the driver on unplug
static volatile uint16_t s_vid, s_pid;       // of the last device to attach
static char s_key[LINKKEY_MAX + 1];          // the link key, "" on an open hub

// The connection being let in. It gets no unit bytes until it proves the key
// (or, on an open hub, until its first line shows it is not a claim).
static int s_pend = -1;
static uint8_t s_nonce[LINKKEY_NONCE_LEN];
static char s_line[LINKKEY_LINE_MAX + 1];
static int s_line_len;
static TickType_t s_pend_until;

/**
 * Runs the USB Host Library's events for good.
 */
static void usb_lib_task(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/**
 * Notes the VID/PID of a device as it attaches (USB host context).
 *
 * :param usb_dev: the device, open only for this call
 */
static void new_dev_cb(usb_device_handle_t usb_dev) {
    const usb_device_desc_t *desc;
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK) {
        s_vid = desc->idVendor;
        s_pid = desc->idProduct;
    }
}

/**
 * Queues bytes from the unit for the TCP client (CDC driver task).
 *
 * :param data: bytes received
 * :param len: how many
 * :param arg: unused
 * :return bool: true, the bytes were taken (dropped if the queue is full)
 */
static bool data_cb(const uint8_t *data, size_t len, void *arg) {
    (void)arg;
    xStreamBufferSend(s_rx, data, len, 0);
    return true;
}

/**
 * Notes an unplug; the unit task closes the handle.
 *
 * :param event: the driver's event
 * :param arg: unused
 */
static void event_cb(const cdc_acm_host_dev_event_data_t *event, void *arg) {
    (void)arg;
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) s_gone = true;
}

/**
 * Prints the hub's state to the console, on change only.
 *
 * :param open: a unit is open
 */
static void report(bool open) {
    static int seen_open = -1, seen_client = -2, seen_key = -1;
    int client = s_client >= 0 ? 1 : 0, key = s_key[0] ? 1 : 0;
    if ((int)open == seen_open && client == seen_client && key == seen_key)
        return;
    seen_open = open;
    seen_client = client;
    seen_key = key;
    printf("{\"evt\":\"hub\",\"port\":%d,\"open\":%d,\"vid\":\"%04x\","
           "\"pid\":\"%04x\",\"baud\":%d,\"client\":%d,\"key\":%d}\n",
           HUB_TCP_PORT, open ? 1 : 0, (unsigned)s_vid, (unsigned)s_pid,
           s_vid == HUB_VID_WCH ? HUB_BAUD_ACE : HUB_BAUD_OTHER, client, key);
}

/**
 * Opens the unit when it appears, at its speed with DTR and RTS up (as a
 * Linux tty open does), and closes it when it goes.
 */
static void unit_task(void *arg) {
    (void)arg;
    const cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = HUB_OPEN_WAIT_MS,
        .out_buffer_size = HUB_XFER_SIZE,
        .in_buffer_size = HUB_XFER_SIZE,
        .event_cb = event_cb,
        .data_cb = data_cb,
        .user_arg = NULL,
    };
    for (;;) {
        if (s_dev == NULL) {
            cdc_acm_dev_hdl_t dev = NULL;
            s_gone = false;
            if (cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &cfg,
                                  &dev) != ESP_OK) {
                report(false);
                continue;                    // the open waited a second
            }
            cdc_acm_line_coding_t lc = {
                .dwDTERate = s_vid == HUB_VID_WCH ? HUB_BAUD_ACE : HUB_BAUD_OTHER,
                .bCharFormat = 0,            // 1 stop bit
                .bParityType = 0,
                .bDataBits = 8,
            };
            cdc_acm_host_line_coding_set(dev, &lc);
            cdc_acm_host_set_control_line_state(dev, true, true);
            xStreamBufferReset(s_rx);
            s_dev = dev;
        } else if (s_gone) {
            cdc_acm_dev_hdl_t dev = s_dev;
            s_dev = NULL;
            cdc_acm_host_close(dev);
        }
        report(s_dev != NULL);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * Sends the unit's bytes to the TCP client as they come.
 */
static void net_tx_task(void *arg) {
    (void)arg;
    static uint8_t buf[HUB_XFER_SIZE];
    for (;;) {
        size_t n = xStreamBufferReceive(s_rx, buf, sizeof buf, portMAX_DELAY);
        if (!n) continue;
        xSemaphoreTake(s_sock_mux, portMAX_DELAY);
        int fd = s_client;
        if (fd >= 0 && send(fd, buf, n, 0) < 0) {
            close(fd);
            s_client = -1;
        }
        xSemaphoreGive(s_sock_mux);
    }
}

/**
 * Sets a client socket up for a serial stream: no Nagle, keepalive.
 *
 * :param fd: the accepted socket
 */
static void tune_client(int fd) {
    int one = 1, idle = 5, intvl = 2, cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof idle);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof intvl);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof cnt);
}

/**
 * Reads the saved link key, if any.
 */
static void load_key(void) {
    nvs_handle_t h;
    s_key[0] = '\0';
    if (nvs_open(HUB_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof s_key;
    if (nvs_get_str(h, HUB_NVS_KEY, s_key, &len) != ESP_OK) s_key[0] = '\0';
    nvs_close(h);
}

/**
 * Saves a link key; the hub asks for it from the next connection on.
 *
 * :param key: the key
 * :return bool: true once it is in flash
 */
static bool save_key(const char *key) {
    nvs_handle_t h;
    if (nvs_open(HUB_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_str(h, HUB_NVS_KEY, key) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    if (ok) snprintf(s_key, sizeof s_key, "%s", key);
    return ok;
}

/**
 * Sends a whole line on a socket.
 *
 * :param fd: the socket
 * :param line: the line, newline included
 */
static void send_line(int fd, const char *line) {
    size_t len = strlen(line), done = 0;
    while (done < len) {
        int n = send(fd, line + done, len - done, 0);
        if (n <= 0) return;
        done += (size_t)n;
    }
}

/**
 * Passes bytes from the client to the unit, if one is open.
 *
 * :param buf: the bytes
 * :param n: how many
 */
static void to_unit(const uint8_t *buf, int n) {
    cdc_acm_dev_hdl_t dev = s_dev;
    if (n > 0 && dev != NULL && !s_gone)
        cdc_acm_host_data_tx_blocking(dev, buf, (size_t)n, HUB_TX_WAIT_MS);
}

/**
 * Lets the pending connection in: it replaces the client, and what it sent
 * after its handshake line goes on to the unit.
 *
 * :param rest: bytes after the handshake line
 * :param n: how many
 */
static void let_in(const char *rest, int n) {
    int fd = s_pend;
    s_pend = -1;
    xSemaphoreTake(s_sock_mux, portMAX_DELAY);
    if (s_client >= 0) close(s_client);
    s_client = fd;
    xSemaphoreGive(s_sock_mux);
    to_unit((const uint8_t *)rest, n);
    report(s_dev != NULL);
}

/**
 * Turns the pending connection away.
 *
 * :param answer: a line to send it first, or NULL
 */
static void turn_away(const char *answer) {
    if (answer) send_line(s_pend, answer);
    close(s_pend);
    s_pend = -1;
}

/**
 * Greets a new connection; it waits as the pending one (a newer one takes
 * the slot).
 *
 * :param fd: the accepted socket
 */
static void greet(int fd) {
    char line[80];
    if (s_pend >= 0) turn_away(NULL);
    s_pend = fd;
    s_line_len = 0;
    if (s_key[0]) esp_fill_random(s_nonce, sizeof s_nonce);
    linkkey_greeting(s_key[0] != '\0', s_nonce, line, sizeof line);
    send_line(fd, line);
    s_pend_until = xTaskGetTickCount()
                   + pdMS_TO_TICKS(s_key[0] ? HUB_KEYED_MS : HUB_OPEN_MS);
}

/**
 * Reads from the pending connection and acts on its first line: the answer
 * to the challenge on a keyed hub, a claim (or the unit's first bytes) on an
 * open one.
 */
static void pending_read(void) {
    static const char no[] = "{\"evt\":\"auth\",\"ok\":0}\n";
    int n = recv(s_pend, s_line + s_line_len, LINKKEY_LINE_MAX - s_line_len, 0);
    if (n <= 0) {
        turn_away(NULL);
        return;
    }
    s_line_len += n;
    s_line[s_line_len] = '\0';
    if (!s_key[0] && !linkkey_maybe_claim(s_line, (size_t)s_line_len)) {
        let_in(s_line, s_line_len);           // the unit's own first bytes
        return;
    }
    char *nl = memchr(s_line, '\n', s_line_len);
    if (!nl) {
        if (s_line_len >= LINKKEY_LINE_MAX) turn_away(no);
        return;
    }
    *nl = '\0';
    int rest = s_line_len - (int)(nl + 1 - s_line);
    char newkey[LINKKEY_MAX + 1];
    switch (linkkey_judge(s_key, s_nonce, s_line, newkey)) {
    case LINKKEY_CLAIM:
        if (!save_key(newkey)) break;
        /* fall through */
    case LINKKEY_OK:
        send_line(s_pend, "{\"evt\":\"auth\",\"ok\":1}\n");
        let_in(nl + 1, rest);
        return;
    }
    turn_away(no);
}

/**
 * Accepts connections, lets in the ones that pass the link key (see the top
 * of this file) and passes the client's bytes to the unit. Bytes for a unit
 * that isn't open are dropped, as a tty with nothing on it would.
 */
static void net_rx_task(void *arg) {
    (void)arg;
    static uint8_t buf[HUB_XFER_SIZE];
    int lfd = -1;
    while (lfd < 0) {
        lfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        struct sockaddr_in a = {
            .sin_family = AF_INET,
            .sin_port = htons(HUB_TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        int one = 1;
        setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        if (lfd < 0 || bind(lfd, (struct sockaddr *)&a, sizeof a) < 0
                || listen(lfd, 2) < 0) {
            if (lfd >= 0) close(lfd);
            lfd = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));  // the network isn't up yet
        }
    }
    for (;;) {
        if (s_pend >= 0 && (int32_t)(xTaskGetTickCount() - s_pend_until) >= 0) {
            if (s_key[0]) turn_away(NULL);    // never answered
            else let_in(s_line, s_line_len);  // an open hub's quiet client
        }
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(lfd, &rd);
        int top = lfd, fd = s_client, pfd = s_pend;
        if (fd >= 0) FD_SET(fd, &rd);
        if (pfd >= 0) FD_SET(pfd, &rd);
        if (fd > top) top = fd;
        if (pfd > top) top = pfd;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
        if (select(top + 1, &rd, NULL, NULL, &tv) <= 0) continue;
        if (FD_ISSET(lfd, &rd)) {
            int nfd = accept(lfd, NULL, NULL);
            if (nfd >= 0) {
                tune_client(nfd);
                greet(nfd);
            }
        }
        if (pfd >= 0 && pfd == s_pend && FD_ISSET(pfd, &rd))
            pending_read();
        if (fd >= 0 && fd == s_client && FD_ISSET(fd, &rd)) {
            int n = recv(fd, buf, sizeof buf, 0);
            if (n <= 0) {
                xSemaphoreTake(s_sock_mux, portMAX_DELAY);
                if (s_client == fd) {
                    close(fd);
                    s_client = -1;
                }
                xSemaphoreGive(s_sock_mux);
                report(s_dev != NULL);
                continue;
            }
            to_unit(buf, n);
        }
    }
}

void hub_start(void) {
    load_key();
    s_rx = xStreamBufferCreate(HUB_RX_BUF, 1);
    s_sock_mux = xSemaphoreCreateMutex();
    const usb_host_config_t host = {
        .skip_phy_setup = false,             // route the S3's PHY to the OTG host
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    if (usb_host_install(&host) != ESP_OK) {
        printf("{\"evt\":\"hub\",\"err\":\"usb_host_install\"}\n");
        return;
    }
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL, HUB_CORE);
    const cdc_acm_host_driver_config_t drv = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 10,
        .xCoreID = HUB_CORE,
        .new_dev_cb = new_dev_cb,
    };
    if (cdc_acm_host_install(&drv) != ESP_OK) {
        printf("{\"evt\":\"hub\",\"err\":\"cdc_acm_host_install\"}\n");
        return;
    }
    xTaskCreatePinnedToCore(unit_task, "hub_unit", 4096, NULL, 5, NULL, HUB_CORE);
    xTaskCreatePinnedToCore(net_tx_task, "hub_tx", 4096, NULL, 6, NULL, HUB_CORE);
    xTaskCreatePinnedToCore(net_rx_task, "hub_rx", 4096, NULL, 6, NULL, HUB_CORE);
}
