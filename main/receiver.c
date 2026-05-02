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

// How long to block re-arming after a trigger fires, in milliseconds.
// Any beam breaks during this window are ignored, so a car with multiple
// gaps/holes counts as a single detection.
// Set to the longest time a car body is expected to take crossing the beam.
// Example: a 30 cm car at 10 km/h takes ~108 ms — 300 ms gives a safe margin.
#ifndef TRIGGER_COOLDOWN_MS
#define TRIGGER_COOLDOWN_MS 300
#endif

#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS 300
#endif

// Set to 1 only when debugging. Logging inside the ESPNOW receive callback
// can add jitter, so it is off by default for timing-critical runs.
#ifndef RX_LOG_EVERY_ESPNOW
#define RX_LOG_EVERY_ESPNOW 0
#endif

// Button pins.
// GPIO4 = Solo mode    (single-gate timing: delta between last two local breaks)
// GPIO5 = Joint mode   (two-gate timing: ESP-NOW RX time vs local gate break)
// GPIO6 = Calibrate    (live voltage display at 2 Hz)
#ifndef SOLO_BTN_GPIO
#define SOLO_BTN_GPIO GPIO_NUM_4
#endif

#ifndef JOINT_BTN_GPIO
#define JOINT_BTN_GPIO GPIO_NUM_5
#endif

#ifndef CALIB_BTN_GPIO
#define CALIB_BTN_GPIO GPIO_NUM_6
#endif

#ifndef BUTTON_POLL_MS
#define BUTTON_POLL_MS 20
#endif

#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 30
#endif

// ---------------------------

// ---------------------------------------------------------------------------
// State machine
//
//  CLEAR        — idle, sensor runs but no action taken. Boot state.
//  SOLO         — single-gate mode: display dt between last two local breaks.
//  JOINT        — two-gate mode: display dt between ESP-NOW RX and local break.
//  CALIBRATING  — display live beam voltage at 2 Hz for alignment.
//
// Transitions:
//   Any state  --[SOLO btn]-->  SOLO         (re-press SOLO in SOLO -> CLEAR)
//   Any state  --[JOINT btn]--> JOINT        (re-press JOINT in JOINT -> CLEAR)
//   Any state  --[CALIB btn]--> CALIBRATING  (re-press CALIB in CALIB -> CLEAR)
// ---------------------------------------------------------------------------

typedef enum {
    APP_STATE_CLEAR = 0,
    APP_STATE_SOLO,
    APP_STATE_JOINT,
    APP_STATE_CALIBRATING
} app_state_t;

typedef enum {
    LCD_UPDATE_NONE = 0,
    LCD_UPDATE_TEXT,
    LCD_UPDATE_TIMING_AND_AVG
} lcd_update_type_t;

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

// --- Joint mode: ESP-NOW RX state ---
static volatile int64_t s_last_rx_time_us   = -1;
static volatile uint32_t s_last_rx_seq       = 0;
static volatile float    s_last_rx_min_v     = 0.0f;
static volatile int64_t  s_last_tx_trigger_us = -1;

// --- Solo mode: last two local gate break timestamps ---
static volatile int64_t s_solo_prev_trigger_us = -1;  // second-to-last break
static volatile int64_t s_solo_last_trigger_us = -1;  // most-recent break

// LCD update state
static TaskHandle_t s_lcd_task_handle = NULL;
static volatile lcd_update_type_t s_lcd_update_type = LCD_UPDATE_NONE;
static char  s_lcd_text[64]  = {0};
static float s_lcd_dt_s      = 0.0f;
static float s_lcd_v_avg     = 0.0f;
static float s_lcd_v_min     = 0.0f;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline float raw_to_volts(uint32_t raw)
{
    const float vref      = 3.3f;
    const float fullscale = (float)((1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U);
    return (raw * vref) / fullscale;
}

static const char *state_to_str(app_state_t st)
{
    switch (st) {
        case APP_STATE_CLEAR:       return "CLEAR";
        case APP_STATE_SOLO:        return "SOLO";
        case APP_STATE_JOINT:       return "JOINT";
        case APP_STATE_CALIBRATING: return "CALIBRATING";
        default:                    return "UNKNOWN";
    }
}

static void reset_runtime_data_locked(void)
{
    // Joint mode
    s_last_rx_time_us    = -1;
    s_last_rx_seq        = 0;
    s_last_rx_min_v      = 0.0f;
    s_last_tx_trigger_us = -1;

    // Solo mode
    s_solo_prev_trigger_us = -1;
    s_solo_last_trigger_us = -1;
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
                                 float    *min_v,
                                 int64_t  *tx_trigger_us)
{
    portENTER_CRITICAL(&s_lock);
    *rx_time_us    = s_last_rx_time_us;
    *seq           = s_last_rx_seq;
    *min_v         = s_last_rx_min_v;
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

// ---------------------------------------------------------------------------
// CSV logging
// ---------------------------------------------------------------------------

static void log_csv_header_once(void)
{
    printf("CSV_HEADER,event,session_id,seq,rx_time_us,trigger_time_us,dt_us,speed_mps,speed_kmh\n");
    fflush(stdout);
}

static void log_csv_row(const char *event,
                        uint32_t    session_id,
                        uint32_t    seq,
                        int64_t     rx_time_us,
                        int64_t     trigger_time_us,
                        int64_t     dt_us,
                        float       speed_mps)
{
    float speed_kmh = speed_mps * 3.6f;
    printf("CSV,%s,%" PRIu32 ",%" PRIu32 ",%" PRIi64 ",%" PRIi64 ",%" PRIi64 ",%.3f,%.3f\n",
           event, session_id, seq,
           rx_time_us, trigger_time_us, dt_us,
           (double)speed_mps, (double)speed_kmh);
    fflush(stdout);
}

// ---------------------------------------------------------------------------
// LCD helpers
// ---------------------------------------------------------------------------

static void lcd_notify_task(void)
{
    TaskHandle_t handle = s_lcd_task_handle;
    if (handle != NULL) {
        xTaskNotifyGive(handle);
    }
}

static void lcd_post_text(const char *msg)
{
    portENTER_CRITICAL(&s_lock);
    strncpy(s_lcd_text, msg, sizeof(s_lcd_text) - 1);
    s_lcd_text[sizeof(s_lcd_text) - 1] = '\0';
    s_lcd_update_type = LCD_UPDATE_TEXT;
    portEXIT_CRITICAL(&s_lock);
    lcd_notify_task();
}

static void lcd_post_timing_and_avg(float dt_s, float v_avg, float v_min)
{
    portENTER_CRITICAL(&s_lock);
    s_lcd_dt_s        = dt_s;
    s_lcd_v_avg       = v_avg;
    s_lcd_v_min       = v_min;
    s_lcd_update_type = LCD_UPDATE_TIMING_AND_AVG;
    portEXIT_CRITICAL(&s_lock);
    lcd_notify_task();
}

static void lcd_show_text_noncritical(const char *msg)
{
    if (s_lcd_task_handle == NULL) {
        ESP_ERROR_CHECK(lcd_print_message(msg));
    } else {
        lcd_post_text(msg);
    }
}

static void lcd_task(void *arg)
{
    (void)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        lcd_update_type_t type;
        char  text[64];
        float dt_s;

        portENTER_CRITICAL(&s_lock);
        type = s_lcd_update_type;
        strncpy(text, s_lcd_text, sizeof(text) - 1);
        text[sizeof(text) - 1] = '\0';
        dt_s              = s_lcd_dt_s;
        s_lcd_update_type = LCD_UPDATE_NONE;
        portEXIT_CRITICAL(&s_lock);

        if (type == LCD_UPDATE_TIMING_AND_AVG) {
            char lcd_msg[64];
            snprintf(lcd_msg, sizeof(lcd_msg), "t:%.3fs", (double)dt_s);
            ESP_ERROR_CHECK(lcd_print_message(lcd_msg));
        } else if (type == LCD_UPDATE_TEXT) {
            ESP_ERROR_CHECK(lcd_print_message(text));
        }
    }
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------

static void app_set_state(app_state_t new_state)
{
    app_state_t old_state;
    uint32_t    session;

    portENTER_CRITICAL(&s_lock);
    old_state = s_app_state;

    if (new_state == APP_STATE_CLEAR) {
        reset_runtime_data_locked();
    }

    // Each active mode gets a fresh session ID when entered.
    if (new_state != APP_STATE_CLEAR &&
        new_state != APP_STATE_CALIBRATING &&
        old_state != new_state)
    {
        reset_runtime_data_locked();
        s_session_id++;
    }

    s_app_state = new_state;
    session     = s_session_id;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "STATE: %s -> %s", state_to_str(old_state), state_to_str(new_state));

    switch (new_state) {
        case APP_STATE_CLEAR:
            lcd_show_text_noncritical("CLEAR");
            log_csv_row("CLEAR", session, 0, -1, -1, -1, 0.0f);
            break;

        case APP_STATE_SOLO:
            lcd_show_text_noncritical("SOLO");
            log_csv_row("SOLO_START", session, 0, -1, -1, -1, 0.0f);
            break;

        case APP_STATE_JOINT:
            lcd_show_text_noncritical("JOINT");
            log_csv_row("JOINT_START", session, 0, -1, -1, -1, 0.0f);
            break;

        case APP_STATE_CALIBRATING:
            lcd_show_text_noncritical("CALIBRATING");
            log_csv_row("CALIBRATE", session, 0, -1, -1, -1, 0.0f);
            break;
    }
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback  (Joint mode only)
// ---------------------------------------------------------------------------

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    (void)info;

    // Only store ESP-NOW data when in Joint mode.
    if (app_get_state() != APP_STATE_JOINT) {
        return;
    }

    int64_t now_us = esp_timer_get_time();

    if (len == (int)sizeof(low_msg_t)) {
        const low_msg_t *m = (const low_msg_t *)data;

        portENTER_CRITICAL_ISR(&s_lock);
        s_last_rx_time_us    = now_us;
        s_last_rx_seq        = m->seq;
        s_last_rx_min_v      = m->min_v;
        s_last_tx_trigger_us = m->ts_us;
        portEXIT_CRITICAL_ISR(&s_lock);

#if RX_LOG_EVERY_ESPNOW
        ESP_LOGI(TAG,
                 "RX msg_type=%u seq=%" PRIu32 " min_v=%.3f tx_ts=%" PRIi64 " rx_time=%" PRIi64,
                 m->msg_type, m->seq, (double)m->min_v, m->ts_us, now_us);
#endif
    } else {
        ESP_LOGW(TAG, "Unexpected len=%d (rx_time_us=%" PRIi64 ")", len, now_us);
    }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

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
        .pin_bit_mask = (1ULL << SOLO_BTN_GPIO)  |
                        (1ULL << JOINT_BTN_GPIO)  |
                        (1ULL << CALIB_BTN_GPIO),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,   // active-low buttons to GND
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io));

    ESP_LOGI(TAG,
             "Buttons: SOLO=%d JOINT=%d CALIB=%d (active-low)",
             (int)SOLO_BTN_GPIO, (int)JOINT_BTN_GPIO, (int)CALIB_BTN_GPIO);
}

// ---------------------------------------------------------------------------
// Button task
//
// Each button toggles its associated mode on/off:
//   Press SOLO  in SOLO        -> CLEAR
//   Press SOLO  in any other   -> SOLO
//   Press JOINT in JOINT       -> CLEAR
//   Press JOINT in any other   -> JOINT
//   Press CALIB in CALIBRATING -> CLEAR
//   Press CALIB in any other   -> CALIBRATING
// ---------------------------------------------------------------------------

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

    int prev_solo  = 1;
    int prev_joint = 1;
    int prev_calib = 1;

    while (1) {
        app_state_t st = app_get_state();

        if (button_pressed_debounced(SOLO_BTN_GPIO, &prev_solo)) {
            app_set_state(st == APP_STATE_SOLO ? APP_STATE_CLEAR : APP_STATE_SOLO);
        }

        if (button_pressed_debounced(JOINT_BTN_GPIO, &prev_joint)) {
            app_set_state(st == APP_STATE_JOINT ? APP_STATE_CLEAR : APP_STATE_JOINT);
        }

        if (button_pressed_debounced(CALIB_BTN_GPIO, &prev_calib)) {
            app_set_state(st == APP_STATE_CALIBRATING ? APP_STATE_CLEAR : APP_STATE_CALIBRATING);
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// Gate trigger handler — called from sensor_task on every confirmed break.
// Implements the Solo and Joint timing logic.
// ---------------------------------------------------------------------------

static void handle_gate_trigger(int64_t t_trigger_us,
                                float   v_min,
                                float   v_max,
                                float   v_avg)
{
    app_state_t st      = app_get_state();
    uint32_t    session = get_session_id();

    ESP_LOGI(TAG,
             "GATE TRIGGERED (V[min=%.3f max=%.3f avg=%.3f]) t_trigger_us=%" PRIi64,
             (double)v_min, (double)v_max, (double)v_avg, t_trigger_us);

    if (st == APP_STATE_SOLO) {
        // ------------------------------------------------------------------
        // Solo mode: measure time between the last two local gate breaks.
        // On the first break there is no previous reference — just store it
        // and show a "waiting" message. On every subsequent break, compute
        // the delta and display it.
        // ------------------------------------------------------------------
        int64_t prev_us;

        portENTER_CRITICAL(&s_lock);
        prev_us                = s_solo_last_trigger_us;
        s_solo_prev_trigger_us = prev_us;
        s_solo_last_trigger_us = t_trigger_us;
        portEXIT_CRITICAL(&s_lock);

        if (prev_us < 0) {
            // No previous break recorded yet.
            ESP_LOGI(TAG, "SOLO: first break recorded, waiting for second");
            lcd_post_text("Waiting...");
            log_csv_row("SOLO_FIRST", session, 0, -1, t_trigger_us, -1, 0.0f);
        } else {
            int64_t dt_us = t_trigger_us - prev_us;

            if (dt_us >= MIN_DT_US) {
                float dt_s = (float)dt_us / 1e6f;

                ESP_LOGI(TAG,
                         "SOLO TIME: prev=%.6fs now=%.6fs dt=%.6fs",
                         (double)prev_us  / 1e6,
                         (double)t_trigger_us / 1e6,
                         (double)dt_s);

                lcd_post_timing_and_avg(dt_s, v_avg, v_min);
                log_csv_row("SOLO_TIME", session, 0, prev_us, t_trigger_us, dt_us, dt_s);
            } else {
                ESP_LOGW(TAG, "SOLO: dt_us=%" PRIi64 " < MIN_DT_US, ignoring", dt_us);
                lcd_post_text("dt too small");
            }
        }

    } else if (st == APP_STATE_JOINT) {
        // ------------------------------------------------------------------
        // Joint mode: delta between ESP-NOW receive time (first gate) and
        // this device's local gate break (second gate).
        // ------------------------------------------------------------------
        int64_t  t_receive_us;
        uint32_t rx_seq;
        float    rx_min_v;
        int64_t  tx_trigger_us_unused;
        get_last_rx_snapshot(&t_receive_us, &rx_seq, &rx_min_v, &tx_trigger_us_unused);

        if (t_receive_us > 0) {
            int64_t dt_us = t_trigger_us - t_receive_us;

            if (dt_us >= MIN_DT_US) {
                float dt_s = (float)dt_us / 1e6f;

                ESP_LOGI(TAG,
                         "JOINT TIME: first_gate_rx=%.6fs receiver_gate=%.6fs dt=%.6fs"
                         " rx_min_v=%.3f receiver_avg_v=%.3f",
                         (double)t_receive_us / 1e6,
                         (double)t_trigger_us / 1e6,
                         (double)dt_s,
                         (double)rx_min_v,
                         (double)v_avg);

                lcd_post_timing_and_avg(dt_s, v_avg, v_min);
                log_csv_row("JOINT_TIME", session, rx_seq,
                            t_receive_us, t_trigger_us, dt_us, dt_s);
            } else {
                ESP_LOGW(TAG, "JOINT: dt_us=%" PRIi64 " < MIN_DT_US, ignoring", dt_us);
                lcd_post_text("dt too small");
            }
        } else {
            ESP_LOGW(TAG, "JOINT: no ESP-NOW message received yet");
            lcd_post_text("No RX msg yet");
        }
    }
    // In CLEAR or CALIBRATING, gate triggers are silently ignored.
}

// ---------------------------------------------------------------------------
// Sensor task
// ---------------------------------------------------------------------------

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

    int     below_count                   = 0;
    bool    armed                         = false;
    int64_t first_below_window_start_us   = -1;
    int64_t cooldown_until_us             = -1;   // re-arm blocked until this time
    app_state_t prev_state                = APP_STATE_CLEAR;

    const uint32_t print_every_windows =
        (PRINT_EVERY_MS <= SENSOR_WINDOW_MS) ? 1u
                                              : (uint32_t)(PRINT_EVERY_MS / SENSOR_WINDOW_MS);
    uint32_t print_div = 0;

    while (1) {
        app_state_t st = app_get_state();

        // ------------------------------------------------------------------
        // CALIBRATING: display live voltage at 2 Hz, no trigger logic.
        // ------------------------------------------------------------------
        if (st == APP_STATE_CALIBRATING) {
            sensor_window_t live_w = {0};
            if (sensor_read_window(window_ticks, &live_w) == ESP_OK && live_w.count > 0) {
                float live_v = raw_to_volts((uint32_t)live_w.avg_raw);
                char  live_msg[32];
                snprintf(live_msg, sizeof(live_msg), "V: %.3f", (double)live_v);
                lcd_post_text(live_msg);
            }
            // Pace to 2 Hz. Guard against underflow if SENSOR_WINDOW_MS >= 500.
            const int sleep_ms = (SENSOR_WINDOW_MS < 500) ? (500 - SENSOR_WINDOW_MS) : 0;
            vTaskDelay(pdMS_TO_TICKS(sleep_ms));

            // Reset trigger state so arming is clean when leaving calibration.
            below_count                 = 0;
            armed                       = false;
            first_below_window_start_us = -1;
            cooldown_until_us           = -1;
            prev_state                  = st;
            continue;
        }

        // ------------------------------------------------------------------
        // CLEAR: sensor keeps running (no warmup needed on mode change) but
        // trigger logic is skipped.
        // ------------------------------------------------------------------
        if (st == APP_STATE_CLEAR) {
            below_count                 = 0;
            armed                       = false;
            first_below_window_start_us = -1;
            cooldown_until_us           = -1;
            prev_state                  = st;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // ------------------------------------------------------------------
        // SOLO or JOINT: active timing modes.
        // ------------------------------------------------------------------

        // On first entry into an active mode, reset and wait for beam-clear arm.
        if (prev_state != st) {
            below_count                 = 0;
            armed                       = false;
            first_below_window_start_us = -1;
            cooldown_until_us           = -1;
            print_div                   = 0;
            ESP_LOGI(TAG, "Entered %s; waiting for beam clear to arm", state_to_str(st));
            prev_state = st;
        }

        int64_t window_start_us = esp_timer_get_time();

        sensor_window_t w   = {0};
        esp_err_t       err = sensor_read_window(window_ticks, &w);
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
                     "window=%dms samples=%" PRIu32
                     " V[min=%.3f max=%.3f avg=%.3f] armed=%d",
                     (int)SENSOR_WINDOW_MS, w.count,
                     (double)v_min, (double)v_max, (double)v_avg,
                     armed ? 1 : 0);
        }

        // Cooldown: after a trigger, block re-arming for TRIGGER_COOLDOWN_MS so
        // that gaps in a multi-hole car body don't produce extra detections.
        if (cooldown_until_us > 0) {
            if (esp_timer_get_time() < cooldown_until_us) {
                // Still in cooldown — discard this window entirely.
                below_count                 = 0;
                first_below_window_start_us = -1;
                continue;
            }
            // Cooldown expired — allow normal arming from here on.
            cooldown_until_us = -1;
            ESP_LOGI(TAG, "Cooldown expired, ready to arm");
        }

        // Arming: wait until the beam is clearly unobstructed before watching
        // for breaks. This prevents false triggers on entry.
        if (!armed) {
            if (v_min > (LOW_THRESH_V + HYST_V)) {
                armed       = true;
                below_count = 0;
                first_below_window_start_us = -1;
                ESP_LOGI(TAG, "Armed (beam clear, v_min=%.3fV)", (double)v_min);
            }
            continue;
        }

        // Consecutive-window debounce.
        bool below = (v_min < LOW_THRESH_V);

        if (below) {
            below_count++;
            if (below_count == 1) {
                first_below_window_start_us = window_start_us;
            }
        } else {
            below_count                 = 0;
            first_below_window_start_us = -1;
        }

        if (below_count >= TRIGGER_CONSECUTIVE_WINDOWS) {
            int64_t t_trigger_us = (first_below_window_start_us >= 0)
                                       ? first_below_window_start_us
                                       : window_start_us;

            handle_gate_trigger(t_trigger_us, v_min, v_max, v_avg);

            // Enter cooldown: discard any breaks for TRIGGER_COOLDOWN_MS.
            // This treats a multi-gap car body as a single detection.
            armed                       = false;
            below_count                 = 0;
            first_below_window_start_us = -1;
            cooldown_until_us           = t_trigger_us
                                          + ((int64_t)TRIGGER_COOLDOWN_MS * 1000LL);
            ESP_LOGI(TAG, "Cooldown started (%d ms)", (int)TRIGGER_COOLDOWN_MS);
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

void app_role_start(void)
{
    ESP_ERROR_CHECK(lcd_init_and_print("Receiver Boot"));
    log_csv_header_once();

    buttons_init();
    rx_init(1);

    // LCD task: lowest priority — never blocks timing-critical paths.
    xTaskCreate(lcd_task,    "lcd_task",    4096, NULL, 3, &s_lcd_task_handle);

    app_set_state(APP_STATE_CLEAR);

    // Sensor task: highest priority — beam timing must not be starved.
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 8, NULL);
    xTaskCreate(button_task, "button_task", 3072, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // CONFIG_ROLE_RX