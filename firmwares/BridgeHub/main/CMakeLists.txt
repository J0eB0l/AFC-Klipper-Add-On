idf_component_register(
    SRCS main.c hub.c linkkey.c wifi.c
    INCLUDE_DIRS .
    PRIV_REQUIRES usb lwip esp_netif esp_wifi esp_event nvs_flash esp_http_server
                  esp_driver_gpio mbedtls)
