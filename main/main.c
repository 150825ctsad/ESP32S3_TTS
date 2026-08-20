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

int app_main()
{
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));

    button_init();
    /* 语音会话：WakeNet 唤醒监听立即开始（网络就绪前唤醒无效，会有提示音） */
    ws_cfg_init();
    /* 非阻塞 WiFi：GOT_IP 后 wifi_cfg 自动拉起 mqtt_task（环境数据上报 + /ws/ 地址） */
    wifi_init();
    vTaskDelay(1000);
    /* 阻塞播放开机提示音（voice_data 分区 WAV） */
    ws_cfg_play_tone();

    while(1){
        vTaskDelay(1000);
    }
    return 0;
}
