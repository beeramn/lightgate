#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_continuous.h"   // adc_channel_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t count;
    uint32_t min_raw;
    uint32_t max_raw;
    float    avg_raw;
} sensor_window_t;

/**
 * Initialize ADC continuous mode for the given channels.
 */
esp_err_t sensor_init(const adc_channel_t *channels, uint8_t channel_num);

/**
 * Start ADC continuous sampling.
 */
esp_err_t sensor_start(void);

/**
 * Block until a full window has been collected and return summary stats.
 * window_ticks: e.g. pdMS_TO_TICKS(500)
 */
esp_err_t sensor_read_window(TickType_t window_ticks, sensor_window_t *out);

/**
 * Optional cleanup.
 */
esp_err_t sensor_stop(void);
esp_err_t sensor_deinit(void);

#ifdef __cplusplus
}
#endif
