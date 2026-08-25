#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_board_init.h"

#include <math.h>
#include "wifi_cfg.h"
#include "button.h"
#include "ws_cfg.h"
#include "led_status.h"

int app_main()
{
    led_status_init();
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));

    button_init();
    /* 语音会话：麦克风 + WakeNet「你好小易」+ 上行组帧，唤醒后播「在」再开 WS */
    ws_cfg_init();
    /* 非阻塞 WiFi：GOT_IP 后 wifi_cfg 自动拉起 mqtt_task（环境数据上报 + /ws/ 地址） */
    wifi_init();
    led_status_boot_done();
    vTaskDelay(1000);
    /* 配网中已播 wificonfig 提示音，避免与开机欢迎音抢 I2S */
    if (!wifi_is_provisioning()) {
        ws_cfg_play_tone();
    }

    while(1){
        vTaskDelay(1000);
    }
    return 0;
}
