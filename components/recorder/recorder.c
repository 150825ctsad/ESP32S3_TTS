/******************************************************************************
 * recorder.c  -- VAD 触发录音 + Base64 编码 + MQTT 上传
 *
 * 工作流程：
 * 1. 持续从麦克风读取 PCM 数据（16kHz 16-bit mono）
 * 2. 计算 RMS 能量，超过阈值则判定为"有声"
 * 3. 检测到声音后录制一段（含预缓冲），Base64 编码后通过 MQTT 上传
 * 4. 静音时跳过，节省带宽
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mbedtls/base64.h"
#include "esp_board_init.h"
#include "bsp_board.h"
#include "mqtt_cfg.h"
#include "recorder.h"

static const char *TAG = "RECORDER";

/* ---- 参数 ---- */
#define REC_SAMPLE_RATE       16000
#define REC_FRAME_MS          20                          /* 每帧 20ms */
#define REC_FRAME_SAMPLES     (REC_SAMPLE_RATE * REC_FRAME_MS / 1000)  /* 320 samples */
#define REC_FRAME_BYTES       (REC_FRAME_SAMPLES * 2)     /* 640 bytes */

#define VAD_THRESHOLD         300                         /* RMS 能量阈值 */
#define VAD_HANGOVER_FRAMES   25                          /* 声音结束后继续录 500ms */
#define VAD_PREFILL_FRAMES    5                           /* 预缓冲 100ms（声音前保留） */
#define VAD_MAX_FRAMES        100                         /* 单段最长 2000ms（100×20ms） */
#define VAD_SILENCE_SKIP      10                          /* 连续 200ms 静音后才回到检测态 */

/* Base64 编码后约 4/3 倍，最大段 100×640×4/3 ≈ 85KB，分片发送 */
#define B64_CHUNK_SIZE        4096                        /* 每次 MQTT 发送的 Base64 块大小 */

/* 预缓冲环形区 */
static int16_t prefill_buf[VAD_PREFILL_FRAMES][REC_FRAME_SAMPLES];
static int prefill_idx = 0;

/* 计算 16-bit PCM 的 RMS 能量 */
static uint32_t calc_rms(const int16_t *pcm, int n)
{
    uint64_t sum = 0;
    for (int i = 0; i < n; i++) {
        int32_t v = pcm[i];
        sum += (uint64_t)(v * v);
    }
    /* 返回 RMS 的整数部分（避免 sqrt 浮点开销） */
    return (uint32_t)(sum / n);
}

static void recorder_task(void *arg)
{
    ESP_LOGI(TAG, "Recorder task started (VAD threshold=%d, max=%dms)",
             VAD_THRESHOLD, VAD_MAX_FRAMES * REC_FRAME_MS);

    int16_t frame[REC_FRAME_SAMPLES];
    /* 录音缓冲区：最大段 + 预缓冲 */
    int16_t *rec_buf = malloc((VAD_MAX_FRAMES + VAD_PREFILL_FRAMES) * REC_FRAME_BYTES);
    if (rec_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc rec buffer (%d bytes)",
                 (VAD_MAX_FRAMES + VAD_PREFILL_FRAMES) * REC_FRAME_BYTES);
        vTaskDelete(NULL);
        return;
    }
    int rec_frames = 0;
    int hangover = 0;
    int silence_count = 0;
    bool recording = false;

    /* Base64 编码缓冲 */
    char *b64_buf = malloc(B64_CHUNK_SIZE + 256);
    if (b64_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc b64 buffer");
        free(rec_buf);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        /* 从麦克风读取一帧 PCM (16-bit 16kHz mono) */
        esp_err_t ret = bsp_get_feed_data(false, frame, REC_FRAME_BYTES);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 维护预缓冲环形区（总是更新，不管是否在录音） */
        memcpy(prefill_buf[prefill_idx], frame, REC_FRAME_BYTES);
        prefill_idx = (prefill_idx + 1) % VAD_PREFILL_FRAMES;

        uint32_t rms = calc_rms(frame, REC_FRAME_SAMPLES);

        if (!recording) {
            /* 检测态：等待声音出现 */
            if (rms > VAD_THRESHOLD) {
                ESP_LOGI(TAG, "VAD triggered (rms=%lu)", (unsigned long)rms);
                recording = true;
                hangover = 0;
                silence_count = 0;
                rec_frames = 0;

                /* 把预缓冲的数据写入录音缓冲（保留声音前 100ms） */
                for (int i = 0; i < VAD_PREFILL_FRAMES; i++) {
                    int idx = (prefill_idx + i) % VAD_PREFILL_FRAMES;
                    memcpy(&rec_buf[rec_frames * REC_FRAME_SAMPLES],
                           prefill_buf[idx], REC_FRAME_BYTES);
                    rec_frames++;
                }
            }
        }

        if (recording) {
            /* 录音态：写入当前帧 */
            if (rec_frames < VAD_MAX_FRAMES + VAD_PREFILL_FRAMES) {
                memcpy(&rec_buf[rec_frames * REC_FRAME_SAMPLES],
                       frame, REC_FRAME_BYTES);
                rec_frames++;
            }

            /* 更新 hangover 计数 */
            if (rms > VAD_THRESHOLD) {
                hangover = VAD_HANGOVER_FRAMES;
                silence_count = 0;
            } else {
                hangover--;
                if (hangover <= 0) {
                    silence_count++;
                    if (silence_count >= VAD_SILENCE_SKIP) {
                        /* 静音确认，结束录音并上传 */
                        int total_bytes = rec_frames * REC_FRAME_BYTES;
                        ESP_LOGI(TAG, "Recording done: %d frames, %d bytes, %d ms",
                                 rec_frames, total_bytes, rec_frames * REC_FRAME_MS);

                        /* 分块 Base64 编码并上传 */
                        int offset = 0;
                        int chunk_frames = (B64_CHUNK_SIZE * 3 / 4) / REC_FRAME_BYTES;
                        if (chunk_frames < 1) chunk_frames = 1;
                        int seq = 0;

                        while (offset < total_bytes) {
                            int remain = total_bytes - offset;
                            int chunk_bytes = remain < chunk_frames * REC_FRAME_BYTES
                                            ? remain : chunk_frames * REC_FRAME_BYTES;

                            size_t b64_len = 0;
                            int enc_ret = mbedtls_base64_encode(
                                (unsigned char *)b64_buf, B64_CHUNK_SIZE + 256,
                                &b64_len,
                                (const unsigned char *)&rec_buf[offset / 2],
                                chunk_bytes);
                            if (enc_ret == 0 && b64_len > 0) {
                                /* 每块前加序号标记，方便云端拼接 */
                                char header[32];
                                int hdr_len = snprintf(header, sizeof(header),
                                                       "{\"seq\":%d,\"total\":%d,\"data\":\"",
                                                       seq, total_bytes);
                                /* 直接用 MQTT 发送 JSON 包装的 Base64 */
                                /* 先发 header，再发 b64，再发尾部 */
                                /* 由于 esp_mqtt_client_publish 只接受单段，
                                 * 这里组装成完整 JSON 再发 */
                                static char *json_msg = NULL;
                                size_t json_size = hdr_len + b64_len + 4;
                                json_msg = realloc(json_msg, json_size);
                                if (json_msg) {
                                    memcpy(json_msg, header, hdr_len);
                                    memcpy(json_msg + hdr_len, b64_buf, b64_len);
                                    memcpy(json_msg + hdr_len + b64_len, "\"}", 2);
                                    json_msg[hdr_len + b64_len + 2] = '\0';
                                    mqtt_publish_audio(json_msg, hdr_len + b64_len + 2);
                                }
                                seq++;
                            }
                            offset += chunk_bytes;
                        }
                        ESP_LOGI(TAG, "Upload done: %d chunks", seq);

                        recording = false;
                        rec_frames = 0;
                        hangover = 0;
                        silence_count = 0;
                    }
                }
            }

            /* 达到最大长度也结束 */
            if (rec_frames >= VAD_MAX_FRAMES + VAD_PREFILL_FRAMES && hangover > 0) {
                hangover = 0;
                silence_count = VAD_SILENCE_SKIP;
            }
        }
    }
    free(rec_buf);
    free(b64_buf);
    vTaskDelete(NULL);
}

void recorder_init(void)
{
    xTaskCreatePinnedToCore(recorder_task, "recorder", 8 * 1024, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "Recorder initialized");
}
