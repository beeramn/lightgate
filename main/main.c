#include "sdkconfig.h"
#include "lcd.h"
#include "esp_log.h"
#if !(CONFIG_ROLE_TX ^ CONFIG_ROLE_RX)
#error "Select exactly one device role"
#endif

void app_role_start(void);

void app_main(void) {
    ESP_ERROR_CHECK(lcd_init_and_print("what up chat"));
    app_role_start();

    // ESP_ERROR_CHECK(lcd_init_and_print("Hello from ESP32-S2"));

    // esp_err_t ret = lcd_init_and_print("Hello");
    // if (ret != ESP_OK) {
    //     ESP_LOGE("MAIN", "lcd_init_and_print failed: %s", esp_err_to_name(ret));
    // } else {
    //     ESP_LOGI("MAIN", "lcd_init_and_print succeeded");
    // }
    // ESP_ERROR_CHECK(lcd_init_and_print("Hello mfs"));


}
