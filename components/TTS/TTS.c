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
/*  tts_speak  -- synchronous playback                               */
/* ================================================================ */
static void tts_speak(const char *text)
{
    if (tts_handle == NULL || text == NULL) return;
    printf("TTS: %s\n", text);

    if (esp_tts_parse_chinese(tts_handle, text)) {
        int total_bytes = 0;
        int len[1] = {0};
        do {
            short *pcm = esp_tts_stream_play(tts_handle, len, TTS_SPEECH_SPEED);
            if (pcm == NULL || len[0] <= 0) break;
            int bytes = len[0] * (int)sizeof(short);
            esp_err_t ret = esp_audio_play(pcm, bytes, portMAX_DELAY);
            if (ret != ESP_OK) printf("TTS: write err %d at %d bytes\n", ret, total_bytes);
            total_bytes += bytes;
        } while (len[0] > 0);
        printf("TTS: wrote %d bytes total, sample[0]=%d\n", total_bytes, total_bytes > 0 ? 1 : 0);
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

/* ================================================================ */
/*  UART input task  -- reads lines from UART, feeds to TTS          */
/* ================================================================ */
void uartTask(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 2 * URAT_BUF_LEN, 0, 0, NULL, 0);
    char data[URAT_BUF_LEN];
    int len = 0;

    printf("\n请输入短语\n");
    while (1) {
        int fd;
        if ((fd = open("/dev/uart/0", O_RDWR)) == -1) {
            ESP_LOGE(TAG, "Cannot open UART");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        uart_vfs_dev_use_driver(0);

        while (1) {
            int s;
            fd_set rfds;
            struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            s = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                break;
            } else if (s == 0) {
                continue;
            } else {
                if (FD_ISSET(fd, &rfds)) {
                    char buf;
                    if (read(fd, &buf, 1) > 0) {
                        if (buf == '\n') {
                            data[len] = '\0';
                            printf("uart input: %s\n", data);
                            tts_speak_async(data);
                            printf("\n请输入短语\n");
                            len = 0;
                        } else if (len < URAT_BUF_LEN) {
                            data[len] = buf;
                            len++;
                        } else {
                            printf("ERROR: text too long\n");
                            len = 0;
                        }
                    } else {
                        ESP_LOGE(TAG, "UART read error");
                        break;
                    }
                } else {
                    ESP_LOGE(TAG, "No FD set in select()");
                    break;
                }
            }
        }
        close(fd);
    }
    vTaskDelete(NULL);
}
