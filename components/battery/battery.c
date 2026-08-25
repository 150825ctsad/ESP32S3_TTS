/******************************************************************************
 * battery.c  -- 充电状态 + 电量百分比（GPIO 检测 + ADC 分压）
 ******************************************************************************/
#include "battery.h"
#include "bsp_board.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BATTERY"

#ifndef GPIO_BATTERY_ADC
#define GPIO_BATTERY_ADC        GPIO_NUM_17
#endif
#ifndef GPIO_CHARGE_DET
#define GPIO_CHARGE_DET         GPIO_NUM_16
#endif
#ifndef GPIO_STDBY_DET
#define GPIO_STDBY_DET          GPIO_NUM_15
#endif
#ifndef CHARGE_ACTIVE_LEVEL
#define CHARGE_ACTIVE_LEVEL     0
#endif
#ifndef BATTERY_DIVIDER
#define BATTERY_DIVIDER         2.0f
#endif
#ifndef BATTERY_EMPTY_MV
#define BATTERY_EMPTY_MV        3300
#endif
#ifndef BATTERY_FULL_MV
#define BATTERY_FULL_MV         4200
#endif

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_adc_cali = NULL;
static adc_channel_t s_adc_ch;
static bool s_inited = false;

esp_err_t battery_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (GPIO_CHARGE_DET != GPIO_NUM_NC) {
        gpio_config_t io = {
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << GPIO_CHARGE_DET,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }
    if (GPIO_STDBY_DET != GPIO_NUM_NC) {
        gpio_config_t io = {
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << GPIO_STDBY_DET,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }

    if (GPIO_BATTERY_ADC != GPIO_NUM_NC) {
        adc_unit_t unit = ADC_UNIT_1;
        esp_err_t err = adc_oneshot_io_to_channel(GPIO_BATTERY_ADC, &unit, &s_adc_ch);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d is not an ADC pin", (int)GPIO_BATTERY_ADC);
        } else {
            adc_oneshot_unit_init_cfg_t ucfg = {
                .unit_id = unit,
            };
            err = adc_oneshot_new_unit(&ucfg, &s_adc);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
                s_adc = NULL;
            } else {
                adc_oneshot_chan_cfg_t ccfg = {
                    .bitwidth = ADC_BITWIDTH_DEFAULT,
                    .atten    = ADC_ATTEN_DB_12,
                };
                err = adc_oneshot_config_channel(s_adc, s_adc_ch, &ccfg);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
                    s_adc = NULL;
                }
            }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            if (s_adc) {
                adc_cali_curve_fitting_config_t cali_cfg = {
                    .unit_id  = unit,
                    .chan     = s_adc_ch,
                    .atten    = ADC_ATTEN_DB_12,
                    .bitwidth = ADC_BITWIDTH_DEFAULT,
                };
                if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
                    s_adc_cali = NULL;
                }
            }
#endif
        }
    }

    s_inited = true;
    ESP_LOGI(TAG, "init ADC GPIO%d chrg GPIO%d stdby GPIO%d divider=%.1f",
             (int)GPIO_BATTERY_ADC, (int)GPIO_CHARGE_DET, (int)GPIO_STDBY_DET,
             (double)BATTERY_DIVIDER);

    xTaskCreatePinnedToCore(battery_task, "battery", 4 * 1024, NULL, 4, NULL, 0);
    return ESP_OK;
}

esp_err_t battery_get_state(bool *charging, bool *full, int *percent)
{
    if (!s_inited) {
        if (charging) *charging = false;
        if (full) *full = false;
        if (percent) *percent = -1;
        return ESP_ERR_INVALID_STATE;
    }

    bool chg = false;
    if (GPIO_CHARGE_DET != GPIO_NUM_NC) {
        chg = (gpio_get_level(GPIO_CHARGE_DET) == CHARGE_ACTIVE_LEVEL);
    }
    if (charging) {
        *charging = chg;
    }

    bool f = false;
    if (GPIO_STDBY_DET != GPIO_NUM_NC) {
        f = (gpio_get_level(GPIO_STDBY_DET) == CHARGE_ACTIVE_LEVEL);
    }
    if (full) {
        *full = f;
    }

    int pct = -1;
    if (s_adc) {
        int raw = 0;
        int sum = 0;
        int n = 0;
        for (int i = 0; i < 8; i++) {
            if (adc_oneshot_read(s_adc, s_adc_ch, &raw) == ESP_OK) {
                sum += raw;
                n++;
            }
        }
        if (n > 0) {
            raw = sum / n;
            int mv = 0;
            if (s_adc_cali == NULL ||
                adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) {
                mv = raw * 3300 / 2048;
            }
            int vbat = (int)((float)mv * BATTERY_DIVIDER);
            ESP_LOGI(TAG, "ADC raw=%d mv=%d vbat=%d mV charging=%d full=%d",
                     raw, mv, vbat, chg ? 1 : 0, f ? 1 : 0);
            if (vbat <= BATTERY_EMPTY_MV) {
                pct = 0;
            } else if (vbat >= BATTERY_FULL_MV) {
                pct = 100;
            } else {
                pct = (vbat - BATTERY_EMPTY_MV) * 100 /
                      (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
            }
        } else {
            ESP_LOGW(TAG, "ADC read failed: raw samples=0, charging=%d full=%d", chg ? 1 : 0, f ? 1 : 0);
        }
    } else {
        ESP_LOGW(TAG, "ADC not initialized; charging=%d full=%d", chg ? 1 : 0, f ? 1 : 0);
    }
    if (percent) {
        *percent = pct;
    }
    return ESP_OK;
}

esp_err_t battery_get(bool *charging, int *percent)
{
    return battery_get_state(charging, NULL, percent);
}


void battery_task(void *arg)
{

    while (1) {
        bool charging = false;
        bool full = false;
        int battery = -1;

        esp_err_t err = battery_get_state(&charging, &full, &battery);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Battery: charging=%s, percent=%d%%, full=%s",
                     charging ? "true" : "false",
                     battery,
                     full ? "true" : "false");
        } else {
            ESP_LOGW(TAG, "Battery read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}