
idf_component_register(
    SRCS "hello_world_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_uart
        esp_driver_gpio
        log
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        mqtt
)