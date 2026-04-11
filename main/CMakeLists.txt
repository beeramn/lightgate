idf_component_register(SRCS "lcd.c" "receiver.c" "sender.c" "main.c" "sensor.c"
    INCLUDE_DIRS "."
    # REQUIRES driver esp_lcd esp_adc nvs_flash esp_netif esp_wifi
)