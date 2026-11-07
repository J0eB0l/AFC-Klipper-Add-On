// BridgeHub: an ESP32-S3 whose own USB port hosts one USB serial unit (a USB
// BoxTurtle, an ACE) and serves it raw over WiFi on TCP port 4001, so AFC can
// reach the unit over the network (serial_port: tcp://<hub ip>:4001).
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "hub.h"
#include "wifi.h"

void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_start();
    hub_start();
}
