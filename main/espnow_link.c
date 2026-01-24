#include "espnow_link.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_wifi_types.h" // wifi_tx_info_t

static const char *TAG = "espnow";

static uint8_t  s_peer_mac[ESP_NOW_ETH_ALEN];
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

esp_err_t espnow_link_init(const uint8_t peer_mac[ESP_NOW_ETH_ALEN],
                           uint8_t wifi_channel)
{
    if (!peer_mac) return ESP_ERR_INVALID_ARG;
    memcpy(s_peer_mac, peer_mac, ESP_NOW_ETH_ALEN);

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
    memcpy(peer.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
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
    return ESP_OK;
}

esp_err_t espnow_link_send_high(uint32_t max_raw)
{
    high_msg_t msg = {
        .msg_type = 1,
        .seq      = s_seq++,
        .max_raw  = max_raw,
        .ts_us    = esp_timer_get_time(),
    };

    esp_err_t err = esp_now_send(s_peer_mac, (uint8_t *)&msg, sizeof(msg));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
    }
    return err;
}
