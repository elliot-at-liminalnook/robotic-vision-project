idf_component_register(
    SRCS "rover_bridge_main.c" "bridge.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_timer esp_driver_uart lwip
)
