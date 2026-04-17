/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#if CONFIG_ROLE_TX  // Only compile this file's logic when Sender is selected

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Sensor module
#include "sensor.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"

#include "esp_wifi_types.h"   // for wifi_tx_info_t
#include "soc/soc_caps.h"     // for SOC_ADC_DIGI_MAX_BITWIDTH

static const char *TAG = "SENDER";

// ---------------- User knobs ----------------

#ifndef SENSOR_WINDOW_MS
#define SENSOR_WINDOW_MS 5
#endif

#ifndef TRIGGER_CONSECUTIVE_WINDOWS
#define TRIGGER_CONSECUTIVE_WINDOWS 2
#endif

#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS 200
#endif

#ifndef LOW_THRESH_V
#define LOW_THRESH_V 1.20f
#endif

#ifndef HYST_V
#define HYST_V 0.10f
#endif

// ADC channel
static adc_channel_t channel[1] = { ADC_CHANNEL_2 };

// ------------------------------------------------

// Receiver STA MAC
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {
    0x80, 0x65, 0x99, 0x5E, 0x28, 0x18
};

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     // 1 = BEAM_BROKEN (LOW)
    uint32_t seq;
    float    min_v;        // voltage metric that triggered (min voltage in window)
    int64_t  ts_us;        // timestamp: FIRST qualifying window start
} low_msg_t;

static uint32_t s_seq = 0;

static void espnow_send_cb(const wifi_tx_info_t *tx_info,
                           esp_now_send_status_t status)
{
    if (tx_info) {
        const uint8_t *da = tx_info->des_addr;
        printf("ESP-NOW send to %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
               da[0], da[1], da[2], da[3], da[4], da[5],
               status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    } else {
        printf("ESP-NOW send -> %s (no tx_info)\n",
               status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    }
}

static void espnow_init_and_add_peer(const uint8_t peer_mac[ESP_NOW_ETH_ALEN], uint8_t wifi_channel)
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

    ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = wifi_channel;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Peer already added");
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_LOGI(TAG, "ESP-NOW ready. Peer added.");
}

static inline float raw_to_volts(uint32_t raw)
{
    const float vref = 3.3f;
    const float fullscale = (float)((1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U);
    return (raw * vref) / fullscale;
}

static void send_beam_broken(float min_v, int64_t t_event_us)
{
    low_msg_t msg = {
        .msg_type = 1,
        .seq = s_seq++,
        .min_v = min_v,
        .ts_us = t_event_us,
    };

    esp_err_t err = esp_now_send(s_peer_mac, (uint8_t *)&msg, sizeof(msg));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
    }
}

void app_role_start(void)
{
    espnow_init_and_add_peer(s_peer_mac, 1);

    ESP_ERROR_CHECK(sensor_init(channel, 1));
    ESP_ERROR_CHECK(sensor_start());

    ESP_LOGI(TAG,
             "TX started: ch=%d window=%dms consec=%d thresh=%.3fV hyst=%.3fV",
             (int)channel[0],
             (int)SENSOR_WINDOW_MS,
             (int)TRIGGER_CONSECUTIVE_WINDOWS,
             (double)LOW_THRESH_V,
             (double)HYST_V);

    const TickType_t window_ticks = pdMS_TO_TICKS(SENSOR_WINDOW_MS);

    const uint32_t print_every_windows =
        (PRINT_EVERY_MS <= SENSOR_WINDOW_MS) ? 1u : (uint32_t)(PRINT_EVERY_MS / SENSOR_WINDOW_MS);
    uint32_t print_div = 0;

    int below_count = 0;
    int64_t first_below_window_start_us = -1;
    bool armed = true;

    while (1) {
        int64_t window_start_us = esp_timer_get_time();

        sensor_window_t w = {0};
        ESP_ERROR_CHECK(sensor_read_window(window_ticks, &w));

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
                    " V[min=%.3f max=%.3f avg=%.3f]",
                    (int)SENSOR_WINDOW_MS,
                    w.count,
                    (double)v_min, (double)v_max, (double)v_avg);
        }

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
                int64_t t_event_us = (first_below_window_start_us >= 0)
                                   ? first_below_window_start_us
                                   : window_start_us;

                ESP_LOGI(TAG, "BEAM BROKEN (v_min=%.3fV) t_event_us=%" PRIi64,
                         (double)v_min, t_event_us);

                send_beam_broken(v_min, t_event_us);

                armed = false;
                below_count = 0;
                first_below_window_start_us = -1;
            }
        } else {
            if (v_min > (LOW_THRESH_V + HYST_V)) {
                armed = true;
                below_count = 0;
                first_below_window_start_us = -1;
                ESP_LOGI(TAG, "Re-armed (v_min=%.3fV)", (double)v_min);
            }
        }
    }
}

#endif // CONFIG_ROLE_TX