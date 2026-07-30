#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_tts.h"
#include "esp_tts_voice_template.h"
#include "esp_board_init.h"
#include "esp_partition.h"
#include "esp_idf_version.h"

#include "TTS.h"
#include "wifi_cfg.h"
#include "relay.h"

/* ================================================================ */
/*  Constants                                                        */
/* ================================================================ */
#define TTS_QUEUE_LEN   10
#define TTS_TEXT_MAX    1024

#define CONFIG_TTS_SPEECH_SPEED 3

/* ================================================================ */
/*  Globals                                                          */
/* ================================================================ */
static esp_tts_handle_t *tts_handle = NULL;
static QueueHandle_t     tts_queue = NULL;

/* ================================================================ */
/*  tts_speak  -- synchronous TTS playback                           */
/* ================================================================ */
static void tts_speak(const char *text)
{
    if (tts_handle == NULL || text == NULL) return;
    printf("TTS: %s\n", text);

    if (esp_tts_parse_chinese(tts_handle, text)) {
        int len[1] = {0};
        do {
            short *pcm = esp_tts_stream_play(tts_handle, len, CONFIG_TTS_SPEECH_SPEED);
            esp_audio_play(pcm, len[0] * 2, portMAX_DELAY);
        } while (len[0] > 0);
    }
    esp_audio_flush();
    esp_tts_stream_reset(tts_handle);
}

/* ================================================================ */
/*  tts_task  -- FreeRTOS task: dequeues text and speaks it          */
/* ================================================================ */
static void tts_task(void *arg)
{
    char text[TTS_TEXT_MAX + 1];
    while (1) {
        if (xQueueReceive(tts_queue, text, portMAX_DELAY) == pdTRUE) {
            tts_speak(text);
        }
    }
}

/* ================================================================ */
/*  tts_speak_async  -- public API, non-blocking                     */
/* ================================================================ */
void tts_speak_async(const char *text)
{
    if (tts_queue == NULL || text == NULL) return;
    char buf[TTS_TEXT_MAX + 1];
    strncpy(buf, text, TTS_TEXT_MAX);
    buf[TTS_TEXT_MAX] = '\0';
    xQueueSend(tts_queue, buf, 0);
}

/* ================================================================ */
/*  tts_init  -- load voice data, create handle + task + queue       */
/* ================================================================ */
static esp_err_t tts_init(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "voice_data");
    if (part == NULL) {
        printf("Couldn't find voice data partition!\n");
        return ESP_ERR_NOT_FOUND;
    }
    printf("voice_data partition size: %lu\n", part->size);

    const void *voicedata;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_partition_mmap_handle_t mmap;
    esp_err_t err = esp_partition_mmap(part, 0, part->size,
                                       ESP_PARTITION_MMAP_DATA, &voicedata, &mmap);
#else
    spi_flash_mmap_handle_t mmap;
    esp_err_t err = esp_partition_mmap(part, 0, part->size,
                                       SPI_FLASH_MMAP_DATA, &voicedata, &mmap);
#endif
    if (err != ESP_OK) {
        printf("Couldn't map voice data partition!\n");
        return err;
    }

    esp_tts_voice_t *voice = esp_tts_voice_set_init(
        &esp_tts_voice_template, (int16_t *)voicedata);
    tts_handle = esp_tts_create(voice);

    tts_queue = xQueueCreate(TTS_QUEUE_LEN, TTS_TEXT_MAX + 1);
    xTaskCreatePinnedToCore(tts_task, "tts_task", 8 * 1024, NULL, 5, NULL, 1);

    return ESP_OK;
}

/* ================================================================ */
/*  app_main  -- entry point: init everything, then let tasks run     */
/* ================================================================ */
int app_main()
{
    relay_state_init();
    relay_init();
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 16));

    if (tts_init() != ESP_OK) return 0;

    wifi_init_sta();
    
    xTaskCreatePinnedToCore(relay_task, "relay", 4 * 1024, NULL, 3, NULL, 1);


    while(1){
        vTaskDelay(1000);
    }
    return 0;
}
