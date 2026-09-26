idf_component_register(
    SRCS
        "main.c"
        "quadmd_protocol.c"
        "quadmd_uart.c"
        "wifi_sta.c"
        "mdns_service.c"
        "web_server.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        driver
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        mdns
        nvs_flash
)
