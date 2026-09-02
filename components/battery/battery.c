/******************************************************************************
 * battery.c  -- 充电状态 + 电量百分比（GPIO 检测 + ADC 分压）
 ******************************************************************************/
#include "battery.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_ll.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BATTERY"

/* 分压后 ADC 低于此值视为没接电池（开路），不要报 0% */
#define VBAT_OPEN_MV            2500
#define ADC_WARMUP              4
#define ADC_SAMPLES             12

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_adc_cali = NULL;
static adc_channel_t s_adc_ch;
static bool s_inited = false;
static int s_pct = -1;
static int s_vbat_mv = -1;

static bool gpio_active(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC) {
        return false;
    }
    return gpio_get_level(pin) == CHARGE_ACTIVE_LEVEL;
}

static int raw_to_pin_mv(int raw)
{
    int mv = 0;
    if (s_adc_cali != NULL &&
        adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK) {
        return mv;
    }
    /* ESP32-S3 默认 12bit，12dB 约 0–3300mV */
    return raw * 3300 / 4095;
}

static int vbat_to_percent(int vbat)
{
    if (vbat < VBAT_OPEN_MV) {
        return -1;
    }
    if (vbat <= BATTERY_EMPTY_MV) {
        return 0;
    }
    if (vbat >= BATTERY_FULL_MV) {
        return 100;
    }
    return (vbat - BATTERY_EMPTY_MV) * 100 /
           (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
}

static esp_err_t adc_read_retry(int *raw)
{
    for (int t = 0; t < 8; t++) {
        if (adc_oneshot_read(s_adc, s_adc_ch, raw) == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    return ESP_FAIL;
}

static void battery_sample_adc(void)
{
    if (s_adc == NULL) {
        s_pct = -1;
        s_vbat_mv = -1;
        return;
    }

    int raw = 0;
    int sum = 0;
    int n = 0;
    for (int i = 0; i < ADC_WARMUP; i++) {
        (void)adc_read_retry(&raw);
    }
    for (int i = 0; i < ADC_SAMPLES; i++) {
        if (adc_read_retry(&raw) == ESP_OK) {
            sum += raw;
            n++;
        }
    }
    if (n <= 0) {
        ESP_LOGW(TAG, "ADC read failed");
        return;
    }

    raw = sum / n;
    int pin_mv = raw_to_pin_mv(raw);
    int vbat = (int)((float)pin_mv * BATTERY_DIVIDER + 0.5f);
    int pct = vbat_to_percent(vbat);

    /* 有效读数做一点平滑，避免上报乱跳 */
    if (pct >= 0 && s_pct >= 0) {
        pct = (s_pct * 3 + pct) / 4;
    }

    s_vbat_mv = vbat;
    s_pct = pct;
    // ESP_LOGI(TAG, "ADC raw=%d pin=%dmV vbat=%dmV pct=%d",
    //          raw, pin_mv, vbat, pct);
}

esp_err_t battery_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (GPIO_CHARGE_DET != GPIO_NUM_NC) {
        gpio_reset_pin(GPIO_CHARGE_DET);
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
        gpio_reset_pin(GPIO_STDBY_DET);
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
        /* 不要 gpio_reset_pin：会打开拉，100k 分压会被拉偏甚至采成 0。
         * GPIO17 = ADC2_CH6。WiFi 占用 ADC2 时 oneshot 可能超时，采样里重试。 */
        adc_unit_t unit = ADC_UNIT_1;
        s_adc_ch = 0;
        esp_err_t map_err = adc_oneshot_io_to_channel(GPIO_BATTERY_ADC, &unit, &s_adc_ch);
        if (map_err != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d is not an ADC pin: %s",
                     (int)GPIO_BATTERY_ADC, esp_err_to_name(map_err));
            s_adc = NULL;
        } else {

            adc_oneshot_unit_init_cfg_t ucfg = {
                .unit_id = unit,
            };
            esp_err_t err = adc_oneshot_new_unit(&ucfg, &s_adc);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
                s_adc = NULL;
            } else {
                adc_oneshot_chan_cfg_t ccfg = {
                    .bitwidth = ADC_BITWIDTH_12,
                    .atten    = ADC_ATTEN_DB_12,
                };
                err = adc_oneshot_config_channel(s_adc, s_adc_ch, &ccfg);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
                    adc_oneshot_del_unit(s_adc);
                    s_adc = NULL;
                } else {
                    adc_ll_set_sample_cycle(255);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            if (s_adc) {
                adc_cali_curve_fitting_config_t cali_cfg = {
                    .unit_id  = unit,
                    .chan     = s_adc_ch,
                    .atten    = ADC_ATTEN_DB_12,
                    .bitwidth = ADC_BITWIDTH_12,
                };
                if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
                    s_adc_cali = NULL;
                    ESP_LOGW(TAG, "ADC cali unavailable, use linear 12-bit");
                }
            }
#endif
            ESP_LOGI(TAG, "init GPIO%d ADC%d_CH%d cali=%d divider=%.1f",
                     (int)GPIO_BATTERY_ADC, (int)unit + 1, (int)s_adc_ch,
                     s_adc_cali ? 1 : 0, (double)BATTERY_DIVIDER);
        }
    }

    s_inited = true;
    battery_sample_adc();

    xTaskCreatePinnedToCore(battery_task, "battery", 3 * 1024, NULL, 4, NULL, 0);
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

    if (charging) {
        *charging = gpio_active(GPIO_CHARGE_DET);
    }
    if (full) {
        *full = gpio_active(GPIO_STDBY_DET);
    }
    if (percent) {
        *percent = s_pct;
    }
    return ESP_OK;
}

esp_err_t battery_get(bool *charging, int *percent)
{
    return battery_get_state(charging, NULL, percent);
}

void battery_task(void *arg)
{
    (void)arg;
    for (;;) {
        battery_sample_adc();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
