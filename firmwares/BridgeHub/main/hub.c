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
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "lwip/sockets.h"

#include "hub.h"

#define HUB_TCP_PORT     4001
#define HUB_CORE         0
#define HUB_RX_BUF       8192      // unit -> TCP, while the client catches up
#define HUB_XFER_SIZE    512
#define HUB_OPEN_WAIT_MS 1000
#define HUB_TX_WAIT_MS   500
#define HUB_VID_WCH      0x1A86    // the ACE's USB serial chip
#define HUB_BAUD_ACE     230400
#define HUB_BAUD_OTHER   115200

static StreamBufferHandle_t s_rx;            // unit -> TCP
static SemaphoreHandle_t s_sock_mux;         // guards s_client
static volatile int s_client = -1;
static volatile cdc_acm_dev_hdl_t s_dev;
static volatile bool s_gone;                 // set by the driver on unplug
static volatile uint16_t s_vid, s_pid;       // of the last device to attach

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
    static int seen_open = -1, seen_client = -2;
    int client = s_client >= 0 ? 1 : 0;
    if ((int)open == seen_open && client == seen_client) return;
    seen_open = open;
    seen_client = client;
    printf("{\"evt\":\"hub\",\"port\":%d,\"open\":%d,\"vid\":\"%04x\","
           "\"pid\":\"%04x\",\"baud\":%d,\"client\":%d}\n",
           HUB_TCP_PORT, open ? 1 : 0, (unsigned)s_vid, (unsigned)s_pid,
           s_vid == HUB_VID_WCH ? HUB_BAUD_ACE : HUB_BAUD_OTHER, client);
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
 * Accepts the TCP client (a new one replaces the old, as ser2net's
 * kickolduser does) and passes its bytes to the unit. Bytes for a unit that
 * isn't open are dropped, as a tty with nothing on it would.
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
                || listen(lfd, 1) < 0) {
            if (lfd >= 0) close(lfd);
            lfd = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));  // the network isn't up yet
        }
    }
    for (;;) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(lfd, &rd);
        int fd = s_client;
        if (fd >= 0) FD_SET(fd, &rd);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        if (select((fd > lfd ? fd : lfd) + 1, &rd, NULL, NULL, &tv) <= 0) continue;
        if (FD_ISSET(lfd, &rd)) {
            int nfd = accept(lfd, NULL, NULL);
            if (nfd >= 0) {
                tune_client(nfd);
                xSemaphoreTake(s_sock_mux, portMAX_DELAY);
                if (s_client >= 0) close(s_client);
                s_client = nfd;
                xSemaphoreGive(s_sock_mux);
                report(s_dev != NULL);
            }
            continue;
        }
        if (fd >= 0 && FD_ISSET(fd, &rd)) {
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
            cdc_acm_dev_hdl_t dev = s_dev;
            if (dev != NULL && !s_gone)
                cdc_acm_host_data_tx_blocking(dev, buf, (size_t)n, HUB_TX_WAIT_MS);
        }
    }
}

void hub_start(void) {
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
