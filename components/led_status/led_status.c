/******************************************************************************
 * led_status.c  -- 红/绿双色灯表示启动、配网、联网、云服务状态
 ******************************************************************************/
#include "led_status.h"
#include "bsp_board.h"
#include "wifi_cfg.h"
#include "mqtt_cfg.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "LED"
#define TICK_MS 50
#define BOOT_HALF_MS 300
#define BLINK_MS 400

#ifndef GPIO_LED_RED
#define GPIO_LED_RED     GPIO_NUM_NC
#endif
#ifndef GPIO_LED_GREEN
#define GPIO_LED_GREEN   GPIO_NUM_NC
#endif
#ifndef LED_ACTIVE_LEVEL
#define LED_ACTIVE_LEVEL 1
#endif

static volatile bool s_booting = true;

static void led_set(int red_on, int green_on)
{
    int on = LED_ACTIVE_LEVEL ? 1 : 0;
    int off = LED_ACTIVE_LEVEL ? 0 : 1;
    if (GPIO_LED_RED != GPIO_NUM_NC) {
        gpio_set_level(GPIO_LED_RED, red_on ? on : off);
    }
    if (GPIO_LED_GREEN != GPIO_NUM_NC) {
        gpio_set_level(GPIO_LED_GREEN, green_on ? on : off);
    }
}

static void led_task(void *arg)
{
    (void)arg;
    int elapsed = 0;
    for (;;) {
        elapsed += TICK_MS;

        if (wifi_is_provisioning()) {
            led_set((elapsed / BLINK_MS) & 1, 0);
        } else if (s_booting) {
            int phase = (elapsed / BOOT_HALF_MS) & 1;
            led_set(phase == 0, phase == 1);
        } else if (!wifi_is_connected()) {
            led_set(1, 0);
        } else if (!mqtt_is_connected()) {
            led_set(0, (elapsed / BLINK_MS) & 1);
        } else {
            led_set(0, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

void led_status_init(void)
{
    if (GPIO_LED_RED == GPIO_NUM_NC && GPIO_LED_GREEN == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "LED pins not configured");
        return;
    }

    gpio_config_t conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = 0,
    };
    if (GPIO_LED_RED != GPIO_NUM_NC) {
        conf.pin_bit_mask |= (1ULL << GPIO_LED_RED);
    }
    if (GPIO_LED_GREEN != GPIO_NUM_NC) {
        conf.pin_bit_mask |= (1ULL << GPIO_LED_GREEN);
    }
    gpio_config(&conf);
    led_set(1, 0);
    s_booting = true;
    xTaskCreatePinnedToCore(led_task, "led_status", 2048, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "LED init RED=GPIO%d GREEN=GPIO%d (boot flash)",
             (int)GPIO_LED_RED, (int)GPIO_LED_GREEN);
}

void led_status_boot_done(void)
{
    s_booting = false;
}
