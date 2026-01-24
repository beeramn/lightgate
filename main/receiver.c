#include "sdkconfig.h"

#if CONFIG_ROLE_RX  // <-- Only compile this file's logic when Receiver is selected

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "RX";

#define LED_PIN GPIO_NUM_4

static TaskHandle_t led_task_handle = NULL;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint32_t seq;
    uint32_t max_raw;
    int64_t  ts_us;
} high_msg_t;

static void gpio_init_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(LED_PIN, 0);
}

static void led_task(void *arg)
{
    while (1) {
        // Wait until ESPNOW callback notifies us
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "LED ON");
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_PIN, 0);
        ESP_LOGI(TAG, "LED OFF");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    (void)info; // silence unused warning (info can be useful if you want MAC, RSSI, etc.)

    if (len == (int)sizeof(high_msg_t)) {
        const high_msg_t *m = (const high_msg_t *)data;

        ESP_LOGI(TAG, "RX msg_type=%u seq=%" PRIu32, m->msg_type, m->seq);

        if (m->msg_type == 1 && led_task_handle) {
            xTaskNotifyGive(led_task_handle);
        }
    } else {
        ESP_LOGW(TAG, "Unexpected len=%d", len);
    }
}

static void rx_init(uint8_t channel)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "Receiver ready on channel %u", channel);
}

/**
 * Role entrypoint called by main.c's app_main().
 * Only exists in RX builds.
 */
void app_role_start(void)
{
    gpio_init_led();

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);

    rx_init(1);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // CONFIG_ROLE_RX
