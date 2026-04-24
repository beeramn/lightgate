#include "sdkconfig.h"

#if CONFIG_ROLE_RX  // Only compile this file's logic when Receiver is selected

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "sensor.h"
#include "lcd.h"
#include "soc/soc_caps.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "RX";

// ---------------------------
// USER TUNABLES
// ---------------------------

#ifndef SENSOR_ADC_CHANNEL
#define SENSOR_ADC_CHANNEL ADC_CHANNEL_2
#endif

#ifndef LOW_THRESH_V
#define LOW_THRESH_V 1.50f
#endif

#ifndef HYST_V
#define HYST_V 0.10f
#endif

#ifndef DISTANCE_METERS
#define DISTANCE_METERS 3.000f
#endif

#ifndef SENSOR_WINDOW_MS
#define SENSOR_WINDOW_MS 5
#endif

#ifndef TRIGGER_CONSECUTIVE_WINDOWS
#define TRIGGER_CONSECUTIVE_WINDOWS 2
#endif

#ifndef MIN_DT_US
#define MIN_DT_US 1000   // 1 ms
#endif

#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS 300
#endif

// Button pins. Change these to match your hardware.
#ifndef START_BTN_GPIO
#define START_BTN_GPIO GPIO_NUM_4
#endif

#ifndef STOP_BTN_GPIO
#define STOP_BTN_GPIO GPIO_NUM_5
#endif

#ifndef CLEAR_BTN_GPIO
#define CLEAR_BTN_GPIO GPIO_NUM_6
#endif

#ifndef BUTTON_POLL_MS
#define BUTTON_POLL_MS 20
#endif

#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 30
#endif

// ---------------------------

typedef enum {
    APP_STATE_CLEAR = 0,
    APP_STATE_RUNNING,
    APP_STATE_STOPPED
} app_state_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Match sender.c exactly
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     // 1 = BEAM_BROKEN
    uint32_t seq;
    float    min_v;
    int64_t  ts_us;        // sender-side trigger timestamp
} low_msg_t;

// App state
static volatile app_state_t s_app_state = APP_STATE_CLEAR;
static volatile uint32_t s_session_id = 0;

// Receiver-side timestamp of last ESPNOW message receive
static volatile int64_t s_last_rx_time_us = -1;
static volatile uint32_t s_last_rx_seq = 0;
static volatile float s_last_rx_min_v = 0.0f;
static volatile int64_t s_last_tx_trigger_us = -1;

// Receiver-side timestamp of last sensor trigger
static volatile int64_t s_last_trigger_time_us = -1;

static inline float raw_to_volts(uint32_t raw)
{
    const float vref = 3.3f;
    const float fullscale = (float)((1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U);
    return (raw * vref) / fullscale;
}

static const char *state_to_str(app_state_t st)
{
    switch (st) {
        case APP_STATE_CLEAR:   return "CLEAR";
        case APP_STATE_RUNNING: return "RUNNING";
        case APP_STATE_STOPPED: return "STOPPED";
        default:                return "UNKNOWN";
    }
}

static void reset_runtime_data_locked(void)
{
    s_last_rx_time_us = -1;
    s_last_rx_seq = 0;
    s_last_rx_min_v = 0.0f;
    s_last_tx_trigger_us = -1;
    s_last_trigger_time_us = -1;
}

static app_state_t app_get_state(void)
{
    app_state_t st;
    portENTER_CRITICAL(&s_lock);
    st = s_app_state;
    portEXIT_CRITICAL(&s_lock);
    return st;
}

static void get_last_rx_snapshot(int64_t *rx_time_us,
                                 uint32_t *seq,
                                 float *min_v,
                                 int64_t *tx_trigger_us)
{
    portENTER_CRITICAL(&s_lock);
    *rx_time_us = s_last_rx_time_us;
    *seq = s_last_rx_seq;
    *min_v = s_last_rx_min_v;
    *tx_trigger_us = s_last_tx_trigger_us;
    portEXIT_CRITICAL(&s_lock);
}

static uint32_t get_session_id(void)
{
    uint32_t session;
    portENTER_CRITICAL(&s_lock);
    session = s_session_id;
    portEXIT_CRITICAL(&s_lock);
    return session;
}

static void log_csv_header_once(void)
{
    printf("CSV_HEADER,event,session_id,seq,rx_time_us,trigger_time_us,dt_us,speed_mps,speed_kmh\n");
    fflush(stdout);
}

static void log_csv_row(const char *event,
                        uint32_t session_id,
                        uint32_t seq,
                        int64_t rx_time_us,
                        int64_t trigger_time_us,
                        int64_t dt_us,
                        float speed_mps)
{
    float speed_kmh = speed_mps * 3.6f;

    printf("CSV,%s,%" PRIu32 ",%" PRIu32 ",%" PRIi64 ",%" PRIi64 ",%" PRIi64 ",%.3f,%.3f\n",
           event,
           session_id,
           seq,
           rx_time_us,
           trigger_time_us,
           dt_us,
           (double)speed_mps,
           (double)speed_kmh);
    fflush(stdout);
}

static void app_set_state(app_state_t new_state)
{
    app_state_t old_state;
    uint32_t session;

    portENTER_CRITICAL(&s_lock);
    old_state = s_app_state;

    if (new_state == APP_STATE_CLEAR) {
        reset_runtime_data_locked();
    }

    if (new_state == APP_STATE_RUNNING && old_state != APP_STATE_RUNNING) {
        reset_runtime_data_locked();
        s_session_id++;
    }

    s_app_state = new_state;
    session = s_session_id;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "STATE: %s -> %s", state_to_str(old_state), state_to_str(new_state));

    switch (new_state) {
        case APP_STATE_CLEAR:
            ESP_ERROR_CHECK(lcd_print_message("CLEAR"));
            log_csv_row("CLEAR", session, 0, -1, -1, -1, 0.0f);
            break;

        case APP_STATE_RUNNING:
            ESP_ERROR_CHECK(lcd_print_message("RUNNING"));
            log_csv_row("START", session, 0, -1, -1, -1, 0.0f);
            break;

        case APP_STATE_STOPPED:
            ESP_ERROR_CHECK(lcd_print_message("STOPPED"));
            log_csv_row("STOP", session, 0, -1, -1, -1, 0.0f);
            break;
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    (void)info;

    if (app_get_state() != APP_STATE_RUNNING) {
        return;
    }

    int64_t now_us = esp_timer_get_time();

    if (len == (int)sizeof(low_msg_t)) {
        const low_msg_t *m = (const low_msg_t *)data;

        portENTER_CRITICAL_ISR(&s_lock);
        s_last_rx_time_us = now_us;
        s_last_rx_seq = m->seq;
        s_last_rx_min_v = m->min_v;
        s_last_tx_trigger_us = m->ts_us;
        portEXIT_CRITICAL_ISR(&s_lock);

        ESP_LOGI(TAG,
                 "RX msg_type=%u seq=%" PRIu32 " min_v=%.3f tx_ts=%" PRIi64 " rx_time=%" PRIi64,
                 m->msg_type, m->seq, (double)m->min_v, m->ts_us, now_us);
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

static void buttons_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << START_BTN_GPIO) |
                        (1ULL << STOP_BTN_GPIO)  |
                        (1ULL << CLEAR_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // assumes active-low buttons to GND
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io));

    ESP_LOGI(TAG,
             "Buttons ready: START=%d STOP=%d CLEAR=%d (active-low)",
             (int)START_BTN_GPIO, (int)STOP_BTN_GPIO, (int)CLEAR_BTN_GPIO);
}

static bool button_pressed_debounced(gpio_num_t pin, int *prev_level)
{
    int curr = gpio_get_level(pin);

    if (curr == 0 && *prev_level == 1) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        curr = gpio_get_level(pin);
        if (curr == 0) {
            *prev_level = 0;
            return true;
        }
    }

    *prev_level = curr;
    return false;
}

static void button_task(void *arg)
{
    (void)arg;

    int prev_start = 1;
    int prev_stop = 1;
    int prev_clear = 1;

    while (1) {
        if (button_pressed_debounced(START_BTN_GPIO, &prev_start)) {
            app_state_t st = app_get_state();
            if (st == APP_STATE_CLEAR || st == APP_STATE_STOPPED) {
                app_set_state(APP_STATE_RUNNING);
            }
        }

        if (button_pressed_debounced(STOP_BTN_GPIO, &prev_stop)) {
            if (app_get_state() == APP_STATE_RUNNING) {
                app_set_state(APP_STATE_STOPPED);
            }
        }

        if (button_pressed_debounced(CLEAR_BTN_GPIO, &prev_clear)) {
            app_set_state(APP_STATE_CLEAR);
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
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
    bool armed = false;
    int64_t first_below_window_start_us = -1;
    app_state_t prev_state = APP_STATE_CLEAR;

    const uint32_t print_every_windows =
        (PRINT_EVERY_MS <= SENSOR_WINDOW_MS) ? 1u : (uint32_t)(PRINT_EVERY_MS / SENSOR_WINDOW_MS);
    uint32_t print_div = 0;

    while (1) {
        app_state_t st = app_get_state();

        if (st != APP_STATE_RUNNING) {
            below_count = 0;
            armed = false;
            first_below_window_start_us = -1;
            prev_state = st;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (prev_state != APP_STATE_RUNNING) {
            below_count = 0;
            armed = false;   // require beam to be clear before arming after Start
            first_below_window_start_us = -1;
            print_div = 0;
            ESP_LOGI(TAG, "Entered RUNNING; waiting for beam clear to arm");
            prev_state = APP_STATE_RUNNING;
        }

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

        float v_min = raw_to_volts(w.min_raw);
        float v_max = raw_to_volts(w.max_raw);
        float v_avg = raw_to_volts((uint32_t)w.avg_raw);

        if (++print_div >= print_every_windows) {
            print_div = 0;
            ESP_LOGI(TAG,
                     "window=%dms samples=%" PRIu32 " V[min=%.3f max=%.3f avg=%.3f] armed=%d",
                     (int)SENSOR_WINDOW_MS, w.count,
                     (double)v_min, (double)v_max, (double)v_avg, armed ? 1 : 0);
        }

        if (!armed) {
            if (v_min > (LOW_THRESH_V + HYST_V)) {
                armed = true;
                below_count = 0;
                first_below_window_start_us = -1;
                ESP_LOGI(TAG, "Armed (beam clear, v_min=%.3fV)", (double)v_min);
            }
            continue;
        }

        bool below = (v_min < LOW_THRESH_V);

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
                t_trigger_us = window_start_us;
            }

            portENTER_CRITICAL(&s_lock);
            s_last_trigger_time_us = t_trigger_us;
            portEXIT_CRITICAL(&s_lock);

            int64_t t_receive_us;
            uint32_t rx_seq;
            float rx_min_v;
            int64_t tx_trigger_us_unused;
            get_last_rx_snapshot(&t_receive_us, &rx_seq, &rx_min_v, &tx_trigger_us_unused);

            ESP_LOGI(TAG,
                     "GATE TRIGGERED (V[min=%.3f max=%.3f avg=%.3f]) t_trigger_us=%" PRIi64 " rx_seq=%" PRIu32,
                     (double)v_min, (double)v_max, (double)v_avg,
                     t_trigger_us, rx_seq);

            if (t_receive_us > 0) {
                int64_t dt_us = t_trigger_us - t_receive_us;

            if (dt_us >= MIN_DT_US) {
                float dt_s = (float)dt_us / 1e6f;
                uint32_t session = get_session_id();

                ESP_LOGI(TAG,
                        "TIME DIFF: first_gate_rx=%.6fs receiver_gate=%.6fs dt=%.6fs, rx_min_v=%.3f",
                        (double)t_receive_us / 1e6,
                        (double)t_trigger_us / 1e6,
                        (double)dt_s,
                        (double)rx_min_v);

                char lcd_msg[64];
                snprintf(lcd_msg, sizeof(lcd_msg), "%.3f s", dt_s);
                ESP_ERROR_CHECK(lcd_print_message(lcd_msg));

                log_csv_row("TIME_DIFF", session, rx_seq, t_receive_us, t_trigger_us, dt_us, dt_s);
            } else {
                    ESP_LOGW(TAG, "dt_us=%" PRIi64 " < MIN_DT_US, ignoring", dt_us);
                    ESP_ERROR_CHECK(lcd_print_message("dt too small"));
                }
            } else {
                ESP_LOGW(TAG, "No ESPNOW message received yet; cannot compute speed");
                ESP_ERROR_CHECK(lcd_print_message("No RX msg yet"));
            }

            armed = false;
            below_count = 0;
            first_below_window_start_us = -1;
        }
    }
}

void app_role_start(void)
{
    ESP_ERROR_CHECK(lcd_init_and_print("Receiver Boot"));
    log_csv_header_once();

    buttons_init();
    rx_init(1);

    app_set_state(APP_STATE_CLEAR);

    xTaskCreate(button_task, "button_task", 3072, NULL, 7, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 6, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // CONFIG_ROLE_RX