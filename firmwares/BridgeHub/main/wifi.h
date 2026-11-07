// BridgeHub WiFi: the saved network, or the setup hotspot (wifi.c).
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#pragma once

/**
 * Joins the saved network, or raises the setup hotspot when none is saved,
 * and watches the BOOT button (held 10 s: forget the network). Needs NVS,
 * esp_netif and the default event loop.
 */
void wifi_start(void);
