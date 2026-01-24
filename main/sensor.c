#include "sensor.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/task.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_continuous.h"

static inline esp_err_t _chk(esp_err_t err, const char *tag, const char *what)
{
    if (err != ESP_OK) {
        ESP_LOGE(tag, "%s failed: %s", what, esp_err_to_name(err));
    }
    return err;
}


// ---------- ADC knobs ----------
#define EXAMPLE_ADC_UNIT            ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH       SOC_ADC_DIGI_MAX_BITWIDTH

#define EXAMPLE_READ_LEN            256
#define SAMPLE_FREQ_HZ              (20 * 1000)
// -------------------------------------------------

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

static const char *TAG = "SENSOR";

static adc_continuous_handle_t s_handle = NULL;
static TaskHandle_t s_task_handle = NULL;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    if (s_task_handle) {
        vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    }
    return (mustYield == pdTRUE);
}

static esp_err_t continuous_adc_init(const adc_channel_t *channels,
                                     uint8_t channel_num,
                                     adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };

    esp_err_t err = adc_continuous_new_handle(&adc_config, &handle);
    if (_chk(err, TAG, "adc_continuous_new_handle") != ESP_OK) return err;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;

    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channels[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "pattern[%d]: unit=%" PRIu8 " ch=%" PRIu8 " atten=%" PRIu8 " bw=%" PRIu8,
                 i,
                 adc_pattern[i].unit,
                 adc_pattern[i].channel,
                 adc_pattern[i].atten,
                 adc_pattern[i].bit_width);
    }

    dig_cfg.adc_pattern = adc_pattern;

    err = adc_continuous_config(handle, &dig_cfg);
    if (_chk(err, TAG, "adc_continuous_config") != ESP_OK) {
        // if config fails, clean up the handle
        adc_continuous_deinit(handle);
        return err;
    }

    *out_handle = handle;
    return ESP_OK;
}

esp_err_t sensor_init(const adc_channel_t *channels, uint8_t channel_num)
{
    if (!channels || channel_num == 0) return ESP_ERR_INVALID_ARG;
    if (s_handle) return ESP_ERR_INVALID_STATE;

    esp_err_t err = continuous_adc_init(channels, channel_num, &s_handle);
    if (err != ESP_OK) return err;

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    err = adc_continuous_register_event_callbacks(s_handle, &cbs, NULL);
    if (_chk(err, TAG, "adc_continuous_register_event_callbacks") != ESP_OK) {
        adc_continuous_deinit(s_handle);
        s_handle = NULL;
        return err;
    }

    return ESP_OK;
}

esp_err_t sensor_start(void)
{
    if (!s_handle) return ESP_ERR_INVALID_STATE;
    return adc_continuous_start(s_handle);
}

esp_err_t sensor_read_window(TickType_t window_ticks, sensor_window_t *out)
{
    if (!s_handle || !out) return ESP_ERR_INVALID_ARG;

    // This function blocks; capture caller task handle so ISR can notify it.
    s_task_handle = xTaskGetCurrentTaskHandle();

    uint32_t count = 0;
    uint32_t min_raw = UINT32_MAX;
    uint32_t max_raw = 0;
    uint64_t sum_raw = 0;

    TickType_t window_start = xTaskGetTickCount();

    uint8_t result[EXAMPLE_READ_LEN];
    uint32_t ret_num = 0;

    while (true) {
        // Wait until driver says there’s data ready
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (true) {
            esp_err_t ret = adc_continuous_read(s_handle, result, sizeof(result), &ret_num, 0);

            if (ret == ESP_OK) {
                for (int i = 0; i < (int)ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);

                    if (chan_num >= SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        continue;
                    }

                    if (data < min_raw) min_raw = data;
                    if (data > max_raw) max_raw = data;
                    sum_raw += data;
                    count++;
                }

                TickType_t now = xTaskGetTickCount();
                if ((now - window_start) >= window_ticks) {
                    out->count = count;
                    out->min_raw = (count > 0) ? min_raw : 0;
                    out->max_raw = (count > 0) ? max_raw : 0;
                    out->avg_raw = (count > 0) ? ((float)sum_raw / (float)count) : 0.0f;
                    return ESP_OK;
                }

                vTaskDelay(pdMS_TO_TICKS(1));
            } else if (ret == ESP_ERR_TIMEOUT) {
                break; 
            } else {
                return ret;
            }
        }
    }
}

esp_err_t sensor_stop(void)
{
    if (!s_handle) return ESP_ERR_INVALID_STATE;
    return adc_continuous_stop(s_handle);
}

esp_err_t sensor_deinit(void)
{
    if (!s_handle) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = adc_continuous_deinit(s_handle);
    s_handle = NULL;
    s_task_handle = NULL;
    return ret;
}
