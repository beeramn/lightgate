#include "sdkconfig.h"

#if CONFIG_ROLE_RX  // Only compile this file's logic when Receiver is selected

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <inttypes.h>

#include "sensor.h"   // uses sensor.c
#include "soc/soc_caps.h"     // for SOC_ADC_DIGI_MAX_BITWIDTH

static const char *TAG = "RX";

#define LED_PIN GPIO_NUM_4

// ---------------------------
// USER TUNABLES
// ---------------------------

#ifndef SENSOR_ADC_CHANNEL
#define SENSOR_ADC_CHANNEL ADC_CHANNEL_2
#endif

// Trigger when beam broken -> voltage goes BELOW this
#ifndef LOW_THRESH_V
#define LOW_THRESH_V 1.50f
#endif

// Re-arm only after voltage rises ABOVE (LOW_THRESH_V + HYST_V)
#ifndef HYST_V
#define HYST_V 0.10f
#endif

#ifndef DISTANCE_METERS
#define DISTANCE_METERS 3.000f
#endif

// Use small window for timing accuracy (e.g., 5ms)
#ifndef SENSOR_WINDOW_MS
#define SENSOR_WINDOW_MS 5
#endif

// Require N consecutive windows below threshold before triggering.
// With 5ms window, N=2 => ~10ms required.
#ifndef TRIGGER_CONSECUTIVE_WINDOWS
#define TRIGGER_CONSECUTIVE_WINDOWS 2
#endif

// ignore triggers if they happen too quickly after message receive (sanity)
#ifndef MIN_DT_US
#define MIN_DT_US 1000   // 1 ms
#endif

// Optional: throttle printing (prints once every PRINT_EVERY_MS)
#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS 300
#endif

// ---------------------------

static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t sensor_task_handle = NULL;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint32_t seq;
    uint32_t max_raw;
    int64_t  ts_us;
} high_msg_t;

// Receiver-side timestamp of last ESPNOW message receive.
static volatile int64_t s_last_rx_time_us = -1;
static volatile uint32_t s_last_rx_seq = 0;

// Receiver-side timestamp of last sensor trigger.
static volatile int64_t s_last_trigger_time_us = -1;

static inline float raw_to_volts(uint32_t raw)
{
    // Simple mapping: 0..(2^bitwidth-1) -> 0..3.3V
    // Absolute accuracy depends on ADC calibration/attenuation, but this is fine
    // for thresholding and consistent behavior between TX/RX.
    const float vref = 3.3f;
    const float fullscale = (float)((1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U);
    return (raw * vref) / fullscale;
}

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
    (void)arg;
    while (1) {
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
    (void)info;

    int64_t now_us = esp_timer_get_time();

    if (len == (int)sizeof(high_msg_t)) {
        const high_msg_t *m = (const high_msg_t *)data;

        s_last_rx_time_us = now_us;
        s_last_rx_seq = m->seq;

        ESP_LOGI(TAG, "RX msg_type=%u seq=%" PRIu32 " (rx_time_us=%" PRIi64 ")",
                 m->msg_type, m->seq, now_us);

        if (m->msg_type == 1 && led_task_handle) {
            xTaskNotifyGive(led_task_handle);
        }
    } else {
        ESP_LOGW(TAG, "Unexpected len=%d (rx_time_us=%" PRIi64 ")", len, now_us);
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

static void sensor_task(void *arg)
{
    (void)arg;

    const adc_channel_t channels[] = { SENSOR_ADC_CHANNEL };
    ESP_ERROR_CHECK(sensor_init(channels, 1));
    ESP_ERROR_CHECK(sensor_start());

    ESP_LOGI(TAG,
             "Sensor started: ch=%d thresh=%.3fV hyst=%.3fV window=%dms consec=%d",
             (int)SENSOR_ADC_CHANNEL,
             (double)LOW_THRESH_V,
             (double)HYST_V,
             (int)SENSOR_WINDOW_MS,
             (int)TRIGGER_CONSECUTIVE_WINDOWS);

    const TickType_t window_ticks = pdMS_TO_TICKS(SENSOR_WINDOW_MS);

    int below_count = 0;
    bool armed = true;

    // Timestamp of the FIRST window that went below threshold
    int64_t first_below_window_start_us = -1;

    // Print throttling
    const uint32_t print_every_windows =
        (PRINT_EVERY_MS <= SENSOR_WINDOW_MS) ? 1u : (uint32_t)(PRINT_EVERY_MS / SENSOR_WINDOW_MS);
    uint32_t print_div = 0;

    while (1) {
        // Timestamp the START of the window
        int64_t window_start_us = esp_timer_get_time();

        sensor_window_t w = {0};
        esp_err_t err = sensor_read_window(window_ticks, &w);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "sensor_read_window failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (w.count == 0) {
            ESP_LOGW(TAG, "No samples collected in window (unexpected).");
            continue;
        }

        // Convert window raw metrics to volts
        float v_min = raw_to_volts(w.min_raw);
        float v_max = raw_to_volts(w.max_raw);
        float v_avg = raw_to_volts((uint32_t)w.avg_raw);

        // Slow periodic status line
        if (++print_div >= print_every_windows) {
            print_div = 0;
            ESP_LOGI(TAG,
                     "window=%dms samples=%" PRIu32 " V[min=%.3f max=%.3f avg=%.3f]",
                     (int)SENSOR_WINDOW_MS, w.count,
                     (double)v_min, (double)v_max, (double)v_avg);
        }

        // "dark" => low voltage in this logic
        bool below = (v_min < LOW_THRESH_V);

        if (armed) {
            if (below) {
                below_count++;
                if (below_count == 1) {
                    first_below_window_start_us = window_start_us;
                }
            } else {
                below_count = 0;
                first_below_window_start_us = -1;
            }

            if (below_count >= TRIGGER_CONSECUTIVE_WINDOWS) {
                int64_t t_trigger_us = first_below_window_start_us;
                if (t_trigger_us < 0) {
                    t_trigger_us = window_start_us; // fallback
                }

                s_last_trigger_time_us = t_trigger_us;

                int64_t t_receive_us = s_last_rx_time_us;

                ESP_LOGI(TAG,
                         "GATE TRIGGERED (V[min=%.3f max=%.3f avg=%.3f]) t_trigger_us=%" PRIi64 " rx_seq=%" PRIu32,
                         (double)v_min, (double)v_max, (double)v_avg,
                         t_trigger_us, s_last_rx_seq);

                if (t_receive_us > 0) {
                    int64_t dt_us = t_trigger_us - t_receive_us;

                    if (dt_us >= MIN_DT_US) {
                        float dt_s = (float)dt_us / 1e6f;
                        float v_mps = DISTANCE_METERS / dt_s;

                        ESP_LOGI(TAG,
                                 "SPEED: dt=%.6fs => %.3f m/s (%.2f km/h)",
                                 (double)dt_s,
                                 (double)v_mps,
                                 (double)(v_mps * 3.6f));
                    } else {
                        ESP_LOGW(TAG, "dt_us=%" PRIi64 " < MIN_DT_US, ignoring", dt_us);
                    }
                }

                armed = false;
                below_count = 0;
                first_below_window_start_us = -1;
            }
        } else {
            // Re-arm when voltage rises above threshold + hysteresis
            if (v_min > (LOW_THRESH_V + HYST_V)) {
                armed = true;
                below_count = 0;
                first_below_window_start_us = -1;
                ESP_LOGI(TAG, "Re-armed (v_min=%.3fV)", (double)v_min);
            }
        }
    }
}

void app_role_start(void)
{
    gpio_init_led();

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);

    rx_init(1);

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 6, &sensor_task_handle);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // CONFIG_ROLE_RX
