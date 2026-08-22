/******************************************************************************
 * TTS.c  -- 离线中文 TTS（云端 PCM 失败时的兜底）
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "TTS.h"
#include "esp_tts.h"
#include "esp_tts_voice_template.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_board_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define TAG               "TTS"
#define TTS_TEXT_MAX      768
#define TTS_SPEECH_SPEED  3
#define TTS_FADE_SAMPLES  128

typedef struct {
    char text[TTS_TEXT_MAX];
} tts_job_t;

static esp_tts_handle_t s_tts = NULL;
static QueueHandle_t s_queue = NULL;
static SemaphoreHandle_t s_done = NULL;
static volatile esp_err_t s_result = ESP_FAIL;
static volatile bool s_tts_abort = false;
static volatile bool s_tts_busy = false;

static void fade_in(int16_t *pcm, int n)
{
    int f = n < TTS_FADE_SAMPLES ? n : TTS_FADE_SAMPLES;
    for (int i = 0; i < f; i++) {
        pcm[i] = (int16_t)((int32_t)pcm[i] * i / f);
    }
}

static void fade_out(int16_t *pcm, int n)
{
    int f = n < TTS_FADE_SAMPLES ? n : TTS_FADE_SAMPLES;
    for (int i = 0; i < f; i++) {
        int idx = n - f + i;
        pcm[idx] = (int16_t)((int32_t)pcm[idx] * (f - i) / f);
    }
}

static esp_err_t tts_speak(const char *text)
{
    if (s_tts == NULL || text == NULL || text[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (!esp_tts_parse_chinese(s_tts, text)) {
        ESP_LOGW(TAG, "parse failed: %s", text);
        return ESP_FAIL;
    }

    int total_bytes = 0;
    int len = 0;
    int16_t *tail = NULL;
    int tail_n = 0;
    bool first = true;

    while (1) {
        if (s_tts_abort) {
            ESP_LOGW(TAG, "aborted");
            break;
        }
        short *pcm = esp_tts_stream_play(s_tts, &len, TTS_SPEECH_SPEED);
        if (pcm == NULL || len <= 0) break;

        if (tail_n > 0) {
            if (s_tts_abort) break;
            esp_audio_play(tail, tail_n * (int)sizeof(int16_t), portMAX_DELAY);
            total_bytes += tail_n * (int)sizeof(int16_t);
        }

        int16_t *tmp = realloc(tail, (size_t)len * sizeof(int16_t));
        if (tmp == NULL) {
            if (s_tts_abort) break;
            if (first) fade_in((int16_t *)pcm, len);
            esp_audio_play((int16_t *)pcm, len * (int)sizeof(int16_t), portMAX_DELAY);
            total_bytes += len * (int)sizeof(int16_t);
            tail_n = 0;
            first = false;
            continue;
        }
        tail = tmp;
        memcpy(tail, pcm, (size_t)len * sizeof(int16_t));
        tail_n = len;
        if (first) {
            fade_in(tail, tail_n);
            first = false;
        }
    }

    if (tail_n > 0 && !s_tts_abort) {
        fade_out(tail, tail_n);
        esp_audio_play(tail, tail_n * (int)sizeof(int16_t), portMAX_DELAY);
        total_bytes += tail_n * (int)sizeof(int16_t);
    }
    free(tail);
    if (!s_tts_abort) {
        esp_audio_flush();
    }
    esp_tts_stream_reset(s_tts);

    if (s_tts_abort) {
        ESP_LOGW(TAG, "stopped for higher-priority WS audio");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "played %d bytes (%d ms): %s",
             total_bytes, total_bytes / 32, text);
    return total_bytes > 0 ? ESP_OK : ESP_FAIL;
}

static void tts_task(void *arg)
{
    (void)arg;
    tts_job_t job;
    for (;;) {
        if (xQueueReceive(s_queue, &job, portMAX_DELAY) == pdTRUE) {
            s_tts_busy = true;
            s_result = tts_speak(job.text);
            s_tts_busy = false;
            xSemaphoreGive(s_done);
        }
    }
}

esp_err_t tts_init(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "tts_data");
    if (part == NULL) {
        ESP_LOGW(TAG, "tts_data partition not found, local TTS disabled");
        return ESP_ERR_NOT_FOUND;
    }

    const void *voicedata = NULL;
    esp_partition_mmap_handle_t mmap;
    esp_err_t err = esp_partition_mmap(part, 0, part->size,
                                       ESP_PARTITION_MMAP_DATA, &voicedata, &mmap);
    if (err != ESP_OK || voicedata == NULL) {
        ESP_LOGE(TAG, "mmap tts_data failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_tts_voice_t *voice = esp_tts_voice_set_init(
        &esp_tts_voice_template, (void *)voicedata);
    if (voice == NULL) {
        ESP_LOGE(TAG, "voice set init failed");
        return ESP_FAIL;
    }
    s_tts = esp_tts_create(voice);
    if (s_tts == NULL) {
        ESP_LOGE(TAG, "esp_tts_create failed");
        return ESP_FAIL;
    }

    s_queue = xQueueCreate(2, sizeof(tts_job_t));
    s_done = xSemaphoreCreateBinary();
    if (s_queue == NULL || s_done == NULL) {
        ESP_LOGE(TAG, "queue/sem create failed");
        return ESP_ERR_NO_MEM;
    }
    xTaskCreatePinnedToCore(tts_task, "tts_task", 8 * 1024, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "local TTS ready (partition %lu bytes)", (unsigned long)part->size);
    return ESP_OK;
}

bool tts_ready(void)
{
    return s_tts != NULL && s_queue != NULL;
}

esp_err_t tts_play(const char *text)
{
    if (!tts_ready()) return ESP_ERR_INVALID_STATE;
    if (text == NULL || text[0] == '\0') return ESP_ERR_INVALID_ARG;

    s_tts_abort = false;
    tts_job_t job;
    memset(&job, 0, sizeof(job));
    snprintf(job.text, sizeof(job.text), "%s", text);
    if (xQueueSend(s_queue, &job, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "queue full, drop");
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(30000)) != pdTRUE) {
        s_tts_abort = true;
        ESP_LOGW(TAG, "play timeout");
        return ESP_ERR_TIMEOUT;
    }
    return s_result;
}

void tts_abort(void)
{
    if (!tts_ready()) {
        return;
    }
    s_tts_abort = true;
}

bool tts_busy(void)
{
    return s_tts_busy;
}
