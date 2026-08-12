/******************************************************************************
 * button.c  -- 音量按键（GPIO4=音量+, GPIO5=音量-，按下接地，低电平有效）
 *
 * 软件消抖 + 长按连发：点按步进 5，按住超过 600ms 后每 200ms 连续调节
 ******************************************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"        /* TickType_t 定义在此，须先于 esp_board_init.h */
#include "freertos/task.h"
#include "button.h"
#include "driver/gpio.h"
#include "esp_board_init.h"
#include "esp_log.h"

#define GPIO_BTN_VOL_UP     GPIO_NUM_4
#define GPIO_BTN_VOL_DOWN   GPIO_NUM_5

#define VOL_STEP            5
#define VOL_MIN             0
#define VOL_MAX             100

#define SCAN_INTERVAL_MS    20          /* 扫描周期 */
#define DEBOUNCE_COUNT      3           /* 消抖确认次数 (60ms) */
#define LONG_PRESS_MS       600         /* 长按判定阈值 */
#define REPEAT_INTERVAL_MS  200         /* 长按连发间隔 */

static const char *TAG = "BUTTON";

typedef struct {
    gpio_num_t gpio;
    int        delta;           /* 该键对应的音量步进（+5 / -5） */
    int        stable_level;    /* 消抖后的稳定电平 */
    int        last_raw;        /* 上次原始采样 */
    int        debounce_cnt;    /* 电平一致计数 */
    int        hold_ms;         /* 持续按住时长 */
    int        repeat_ms;       /* 距上次连发的时间 */
} btn_t;

static void adjust_volume(int delta)
{
    int vol = 0;
    if (esp_audio_get_play_vol(&vol) != ESP_OK) return;

    vol += delta;
    if (vol > VOL_MAX) vol = VOL_MAX;
    if (vol < VOL_MIN) vol = VOL_MIN;
    esp_audio_set_play_vol(vol);
    ESP_LOGI(TAG, "Volume: %d", vol);
}

static void button_task(void *arg)
{
    btn_t btns[] = {
        { .gpio = GPIO_BTN_VOL_UP,   .delta = +VOL_STEP,
          .stable_level = 1, .last_raw = 1, .debounce_cnt = 0, .hold_ms = 0, .repeat_ms = 0 },
        { .gpio = GPIO_BTN_VOL_DOWN, .delta = -VOL_STEP,
          .stable_level = 1, .last_raw = 1, .debounce_cnt = 0, .hold_ms = 0, .repeat_ms = 0 },
    };

    while (1) {
        for (int i = 0; i < sizeof(btns) / sizeof(btns[0]); i++) {
            btn_t *b = &btns[i];
            int raw = gpio_get_level(b->gpio);

            if (raw != b->last_raw) {
                /* 电平变化，重新计数 */
                b->last_raw = raw;
                b->debounce_cnt = 0;
            } else if (b->debounce_cnt < DEBOUNCE_COUNT) {
                /* 连续采样一致，达到阈值则确认电平有效 */
                if (++b->debounce_cnt == DEBOUNCE_COUNT) {
                    if (b->stable_level == 1 && raw == 0) {
                        /* 确认按下（1→0 边沿）：立即调节一次 */
                        adjust_volume(b->delta);
                        b->hold_ms = 0;
                        b->repeat_ms = 0;
                    }
                    b->stable_level = raw;
                }
            } else if (b->stable_level == 0) {
                /* 按住状态：超过长按阈值后开始连发 */
                b->hold_ms += SCAN_INTERVAL_MS;
                if (b->hold_ms >= LONG_PRESS_MS) {
                    b->repeat_ms += SCAN_INTERVAL_MS;
                    if (b->repeat_ms >= REPEAT_INTERVAL_MS) {
                        b->repeat_ms = 0;
                        adjust_volume(b->delta);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

void button_init(void)
{
    gpio_config_t conf = {
        .mode         = GPIO_MODE_INPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << GPIO_BTN_VOL_UP) | (1ULL << GPIO_BTN_VOL_DOWN),
    };
    gpio_config(&conf);

    xTaskCreatePinnedToCore(button_task, "button", 2 * 1024, NULL, 4, NULL, 0);
    ESP_LOGI(TAG, "Button init: VOL_UP=GPIO%d, VOL_DOWN=GPIO%d", GPIO_BTN_VOL_UP, GPIO_BTN_VOL_DOWN);
}
