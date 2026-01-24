#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_now.h"

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;   // 1 = HIGH
    uint32_t seq;
    uint32_t max_raw;
    int64_t  ts_us;
} high_msg_t;

esp_err_t espnow_link_init(const uint8_t peer_mac[ESP_NOW_ETH_ALEN],
                           uint8_t wifi_channel);

esp_err_t espnow_link_send_high(uint32_t max_raw);
