#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_board_init.h"

#include <math.h>
#include "TTS.h"
#include "wifi_cfg.h"
#include "relay.h"

/* Quick 1 kHz test tone — purely I2S path verification, no TTS */
static void test_beep(void)
{
    const int sr = 16000;
    const int dur_ms = 150;
    const int freq = 1000;
    int nsamp = sr * dur_ms / 1000;
    int16_t *buf = malloc(nsamp * sizeof(int16_t));
    if (!buf) return;
    for (int i = 0; i < nsamp; i++)
        buf[i] = (int16_t)(20000.0 * sinf(2.0f * 3.14159265f * freq * i / sr));
    printf("BEEP: writing %d samples at %d Hz\n", nsamp, freq);
    esp_err_t r = esp_audio_play(buf, nsamp * sizeof(int16_t), portMAX_DELAY);
    printf("BEEP: write ret=%d\n", r);
    esp_audio_flush();
    free(buf);
}

int app_main()
{
    relay_init();
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));

    //test_beep();  /* ← direct I2S test (bypasses TTS) */

    if (tts_init() != ESP_OK) return 0;

    wifi_init();
    xTaskCreatePinnedToCore(uartTask, "uart_in", 4 * 1024, NULL, 5, NULL, 0);

    /* 非阻塞播报启动提示 —— TTS 任务已就绪 */
    tts_speak_async("欢迎使用乐鑫语音合成");

    while(1){
        vTaskDelay(1000);
    }
    return 0;
}
