/******************************************************************************
 * TTS.c  -- Chinese TTS engine wrapper + UART input task
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_tts.h"
#include "esp_tts_voice_template.h"
#include "esp_board_init.h"
#include "esp_partition.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "driver/uart_vfs.h"

#include "TTS.h"

/* ================================================================ */
/*  Constants                                                        */
/* ================================================================ */
#define TTS_QUEUE_LEN         10
#define TTS_TEXT_MAX          1024
#define TTS_SPEECH_SPEED      3
#define TAG                   "TTS"

/* ================================================================ */
/*  Globals                                                          */
/* ================================================================ */
static esp_tts_handle_t *tts_handle = NULL;
static QueueHandle_t     tts_queue = NULL;

/* ================================================================ */
/*  Fade in/out -- 消除每段语音首尾的咔哒爆音 (8ms @16kHz)           */
/* ================================================================ */
#define TTS_FADE_SAMPLES   128

static void fade_in(int16_t *pcm, int n)
{
    int f = n < TTS_FADE_SAMPLES ? n : TTS_FADE_SAMPLES;
    for (int i = 0; i < f; i++)
        pcm[i] = (int16_t)((int32_t)pcm[i] * i / f);
}

static void fade_out(int16_t *pcm, int n)
{
    int f = n < TTS_FADE_SAMPLES ? n : TTS_FADE_SAMPLES;
    for (int i = 0; i < f; i++) {
        int idx = n - f + i;
        pcm[idx] = (int16_t)((int32_t)pcm[idx] * (f - i) / f);
    }
}

/* ================================================================ */
/*  tts_speak  -- synchronous playback                               */
/* ================================================================ */
static void tts_speak(const char *text)
{
    if (tts_handle == NULL || text == NULL) return;
    printf("TTS: %s\n", text);

    if (esp_tts_parse_chinese(tts_handle, text)) {
        int total_bytes = 0;
        int len[1] = {0};
        int16_t *tail = NULL;      /* 缓存最近一块：仅最后一块需要淡出 */
        int tail_len = 0;
        bool first = true;

        while (1) {
            short *pcm = esp_tts_stream_play(tts_handle, len, TTS_SPEECH_SPEED);
            if (pcm == NULL || len[0] <= 0) break;

            /* 上一块不是结尾，直接写出 */
            if (tail_len > 0) {
                esp_err_t ret = esp_audio_play(tail, tail_len * (int)sizeof(short), portMAX_DELAY);
                if (ret != ESP_OK) printf("TTS: write err %d at %d bytes\n", ret, total_bytes);
                total_bytes += tail_len * (int)sizeof(short);
            }

            /* stream_play 的内部缓冲下次调用即失效，必须拷贝 */
            int16_t *tmp = realloc(tail, len[0] * sizeof(short));
            if (tmp == NULL) {       /* 内存不足则放弃淡出，直接写当前块 */
                esp_audio_play(pcm, len[0] * (int)sizeof(short), portMAX_DELAY);
                total_bytes += len[0] * (int)sizeof(short);
                tail_len = 0;
                first = false;
                continue;
            }
            tail = tmp;
            memcpy(tail, pcm, len[0] * sizeof(short));
            tail_len = len[0];

            if (first) {
                fade_in(tail, tail_len);
                first = false;
            }
        }

        /* 缓存的最后一块做淡出后写出 */
        if (tail_len > 0) {
            fade_out(tail, tail_len);
            esp_err_t ret = esp_audio_play(tail, tail_len * (int)sizeof(short), portMAX_DELAY);
            if (ret != ESP_OK) printf("TTS: write err %d at %d bytes\n", ret, total_bytes);
            total_bytes += tail_len * (int)sizeof(short);
        }
        free(tail);
        printf("TTS: wrote %d bytes (%d ms)\n", total_bytes, total_bytes / 2 / 16);
    } else {
        printf("TTS: parse failed!\n");
    }
    esp_err_t flush_ret = esp_audio_flush();
    if (flush_ret != ESP_OK) printf("TTS: flush err %d\n", flush_ret);
    esp_tts_stream_reset(tts_handle);
}

/* ================================================================ */
/*  tts_task  -- dequeue and speak                                   */
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
/*  Public API                                                       */
/* ================================================================ */

/* ---- non-blocking speak ---- */
void tts_speak_async(const char *text)
{
    if (tts_queue == NULL || text == NULL) return;
    char buf[TTS_TEXT_MAX + 1];
    strncpy(buf, text, TTS_TEXT_MAX);
    buf[TTS_TEXT_MAX] = '\0';
    xQueueSend(tts_queue, buf, 0);
}

/* ---- one-time init: load voice, create queue + task ---- */
esp_err_t tts_init(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "voice_data");
    if (part == NULL) {
        printf("Could not find voice_data partition!\n");
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
        printf("Could not map voice_data partition!\n");
        return err;
    }

    esp_tts_voice_t *voice = esp_tts_voice_set_init(
        &esp_tts_voice_template, (int16_t *)voicedata);
    tts_handle = esp_tts_create(voice);

    tts_queue = xQueueCreate(TTS_QUEUE_LEN, TTS_TEXT_MAX + 1);
    xTaskCreatePinnedToCore(tts_task, "tts_task", 8 * 1024, NULL, 5, NULL, 1);

    return ESP_OK;
}
