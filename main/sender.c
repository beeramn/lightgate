/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#if CONFIG_ROLE_TX  // <-- Only compile this file's logic when Sender is selected

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
#define READ_INTERVAL_TICKS         pdMS_TO_TICKS(500)  // 0.5s window

// Software thresholds in raw ADC units for rn 
#define HIGH_THRESH_RAW             0x258
#define LOW_THRESH_RAW              0x020

// Choose channel to get sensor data
static adc_channel_t channel[1] = { ADC_CHANNEL_2 };
// ------------------------------------------------

// Receiver STA MAC (the other ESP32-S2)
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {
    0x80, 0x65, 0x99, 0x5E, 0x28, 0xD4
};

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     // 1 = HIGH
    uint32_t seq;
    uint32_t max_raw;      // send the max raw value
    int64_t  ts_us;        // timestamp
} high_msg_t;

static uint32_t s_seq = 0;

// Optional: state to avoid repeatedly calling high/low every print window
typedef enum {
    LEVEL_MID = 0,
    LEVEL_LOW,
    LEVEL_HIGH,
} level_t;

static void espnow_send_cb(const wifi_tx_info_t *tx_info,
                           esp_now_send_status_t status)
{
    // tx_info can be NULL in some cases, so be defensive.
    if (tx_info) {
        const uint8_t *da = tx_info->des_addr;   // destination MAC (peer)
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
    // NVS is required by Wi-Fi in ESP-IDF
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

    // ESPNOW STA only
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Both devices must be on the same Wi-Fi channel for ESP-NOW to work reliably
    ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // Add peer
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = wifi_channel;   // must match the channel being set
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Peer already added");
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_LOGI(TAG, "ESP-NOW ready. Peer added.");
}

static void low_thresh(void)
{
    // TODO: add low threshold behavior if needed
}

static void high_thresh(uint32_t max_raw)
{
    high_msg_t msg = {
        .msg_type = 1,
        .seq = s_seq++,
        .max_raw = max_raw,
        .ts_us = esp_timer_get_time()
    };

    esp_err_t err = esp_now_send(s_peer_mac, (uint8_t *)&msg, sizeof(msg));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
    }
}

/**
 * Role entrypoint called by main.c's app_main().
 * Only exists in TX builds.
 */
void app_role_start(void)
{
    // ESP-NOW init
    espnow_init_and_add_peer(s_peer_mac, 1);

    // Sensor init/start
    ESP_ERROR_CHECK(sensor_init(channel, 1));
    ESP_ERROR_CHECK(sensor_start());

    const char unit_str[] = "ADC_UNIT_1";

    // Precompute for voltage conversion
    const float vref = 3.3f;
    const float adc_fullscale = (float)((1U << SOC_ADC_DIGI_MAX_BITWIDTH) - 1U);

    // Track threshold state (prevents repeated triggers)
    level_t last_level = LEVEL_MID;

    while (1) {
        sensor_window_t w = {0};
        ESP_ERROR_CHECK(sensor_read_window(READ_INTERVAL_TICKS, &w));

        if (w.count > 0) {
            float v_min = (w.min_raw * vref) / adc_fullscale;
            float v_max = (w.max_raw * vref) / adc_fullscale;
            float v_avg = (w.avg_raw * vref) / adc_fullscale;

            ESP_LOGI(TAG,
                     "Unit:%s Ch:%" PRIu32 " window=500ms samples=%" PRIu32
                     " raw[min=%" PRIu32 " max=%" PRIu32 " avg=%.1f] V[min=%.3f max=%.3f avg=%.3f]",
                     unit_str,
                     (uint32_t)channel[0],
                     w.count,
                     w.min_raw, w.max_raw, w.avg_raw,
                     v_min, v_max, v_avg);

            // Decide current level based on window (uses max/min)
            level_t level_now = LEVEL_MID;
            if (w.max_raw >= HIGH_THRESH_RAW) {
                level_now = LEVEL_HIGH;
            } else if (w.min_raw <= LOW_THRESH_RAW) {
                level_now = LEVEL_LOW;
            }

            // Trigger actions only on changes (prevents spam)
            if (level_now != last_level) {
                last_level = level_now;

                if (level_now == LEVEL_HIGH) {
                    ESP_LOGI(TAG, "HIGH (software threshold)");
                    high_thresh(w.max_raw);
                } else if (level_now == LEVEL_LOW) {
                    ESP_LOGI(TAG, "LOW (software threshold)");
                    low_thresh();
                } else {
                    ESP_LOGI(TAG, "MID (back in range)");
                }
            }
        } else {
            ESP_LOGW(TAG, "No samples collected in window (unexpected).");
        }
    }
}

#endif // CONFIG_ROLE_TX
