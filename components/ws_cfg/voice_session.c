/* voice_session.c  -- 语音会话状态机
 *
 * 唤醒词 → WS 连接（唤醒才连，会话结束即断）→ 上行 PCM（云端 ASR）
 * → 下行 TTS PCM → 本地播放 → 回 IDLE 重新监听唤醒词。
 *
 * 状态机：
 *   IDLE ──wakenet命中──▶ CONNECTING ──连接成功──▶ STREAMING ──VAD静音/10s上限──▶ WAIT_TTS
 *     ▲                       │(3s超时/失败: 提示音) │                          │(5s超时)
 *     │                       │                      ▼                          │
 *     └── done/error/断线 ◀── PLAYING ◀──────── 收到首字节 PCM ◀───────────────┘
 *
 * 任务划分：
 *   session_task: mic 读取（3ch 交错 → mono）+ WakeNet + 上行组帧 + VAD + 状态机
 *   player_task : rb_read → esp_audio_play（rb_reset 仅本任务执行）
 *
 * 下行缓冲（ring）排空握手：
 *   正常结束（done）  → rb_done_write   → player 播完剩余后 rb_read 返回 RB_DONE → reset
 *   强制结束（异常）  → rb_unblock_reader → player rb_read 返回 RB_TIMEOUT → reset
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ws_cfg.h"
#include "ws_cfg_internal.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "esp_board_init.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "ringbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define TAG "VOICE_SESSION"

#define WAKE_MODEL_NAME      "wn9s_hilexin"   /* 唤醒词：嗨，乐鑫（无 PSRAM 用小模型） */

/* 音频参数 */
#define MIC_CHANNELS         3                /* KORVO-2 输出 3 通道交错 [M,N,R] */
#define PCM_FRAME_SAMPLES    512              /* 上行 PCM 帧：512 采样 = 1024B = 32ms */

/* 环形缓冲：下行 TTS 音频（WS 回调写 → player 读） */
#define RING_BLOCK_SIZE      1024
#define RING_N_BLOCKS        32               /* 32KB ≈ 1s @16kHz mono */

/* VAD（沿用 recorder 参数，帧长 ~15ms） */
#define VAD_RMS_THRESHOLD    300              /* RMS 阈值 */
#define VAD_HANGOVER         32               /* 静音挂尾帧数 ≈ 480ms */

/* 会话超时 */
#define CONNECT_TIMEOUT_MS   3000
#define STREAM_MAX_MS        10000
#define WAIT_TTS_TIMEOUT_MS  5000
#define PLAY_MAX_MS          30000

typedef enum {
    SESS_IDLE = 0,
    SESS_CONNECTING,
    SESS_STREAMING,
    SESS_WAIT_TTS,
    SESS_PLAYING,
} sess_state_t;

static EventGroupHandle_t s_evt = NULL;
static ringbuf_handle_t s_ring = NULL;

static const esp_wn_iface_t *s_wn = NULL;
static model_iface_data_t *s_wn_data = NULL;
static int s_wn_chunk = 0;                    /* wakenet 每次喂入的采样数（运行时查询） */

static volatile sess_state_t s_state = SESS_IDLE;
static char s_device[13] = "000000000000";

/* ================================================================ */
/*  会话收尾                                                         */
/* ================================================================ */

static void session_teardown(bool normal)
{
    if (normal) {
        /* done：rb_done_write → player 播完 ring 剩余后返回 RB_DONE → 自行 reset */
        rb_done_write(s_ring);
        ESP_LOGI(TAG, "Draining playback");
    } else {
        /* 异常：unblock → player 立即返回 RB_TIMEOUT → 自行 reset（丢弃残留） */
        rb_unblock_reader(s_ring);
        ESP_LOGW(TAG, "Abort playback");
    }
    ws_cfg_disconnect();
    s_state = SESS_IDLE;
    ESP_LOGI(TAG, "Session teardown -> IDLE");
}

/* ================================================================ */
/*  player_task：下行 PCM 流式播放                                   */
/* ================================================================ */

static void player_task(void *arg)
{
    char buf[RING_BLOCK_SIZE];
    for (;;) {
        int n = rb_read(s_ring, buf, sizeof(buf), portMAX_DELAY);
        if (n <= 0) {
            /* RB_DONE（正常结束）或 RB_TIMEOUT/RB_ABORT（强制结束） */
            rb_reset(s_ring);                 /* 本任务独占 reset，准备下一会话 */
            continue;
        }
        esp_audio_play((int16_t *)buf, n, portMAX_DELAY);
    }
}

/* ================================================================ */
/*  session_task：唤醒监听 + 会话状态机                              */
/* ================================================================ */

static void session_task(void *arg)
{
    int16_t frame[240 * MIC_CHANNELS];   /* 一次 bsp_get_feed_data 的 3ch 交错数据 */
    int16_t mono[512];                   /* 单声道 */
    int16_t wn_buf[1024];                /* wakenet 累积缓冲 */
    int wn_n = 0;
    int16_t tx_frame[PCM_FRAME_SAMPLES]; /* 上行组帧 */
    int tx_n = 0;
    int hangover = 0;
    TickType_t t_connect_start = 0, t_stream_start = 0, t_wait_start = 0, t_play_start = 0;

    for (;;) {
        int ret = esp_get_feed_data(false, frame, sizeof(frame));
        if (ret <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* 3ch 交错 [M,N,R] → 取第一通道 M 为 mono */
        int nsamp = ret / (2 * MIC_CHANNELS);
        if (nsamp > (int)(sizeof(mono) / sizeof(mono[0]))) nsamp = sizeof(mono) / sizeof(mono[0]);
        for (int i = 0; i < nsamp; i++) mono[i] = frame[3 * i];

        switch (s_state) {

        /* ---------- 空闲：喂 WakeNet ---------- */
        case SESS_IDLE:
            if (s_wn && s_wn_data) {
                for (int i = 0; i < nsamp && wn_n < (int)(sizeof(wn_buf) / sizeof(wn_buf[0])); i++) {
                    wn_buf[wn_n++] = mono[i];
                }
                if (wn_n >= s_wn_chunk) {
                    wn_n = 0;
                    if (s_wn->detect(s_wn_data, wn_buf) == WAKENET_DETECTED) {
                        ESP_LOGI(TAG, "!!! Wake word detected !!!");
                        if (!ws_cfg_has_uri()) {
                            ESP_LOGW(TAG, "No ws uri yet, tone and ignore");
                            ws_cfg_play_tone();
                            break;
                        }
                        xEventGroupClearBits(s_evt,
                            WS_EVT_CONNECTED | WS_EVT_DISCONNECTED |
                            WS_EVT_TTS_DONE | WS_EVT_TTS_ERROR);
                        if (ws_cfg_connect() == ESP_OK) {
                            s_state = SESS_CONNECTING;
                            t_connect_start = xTaskGetTickCount();
                        } else {
                            ws_cfg_play_tone();
                        }
                    }
                }
            }
            break;

        /* ---------- 连接中：等 CONNECTED / 超时 ---------- */
        case SESS_CONNECTING:
            if (xEventGroupGetBits(s_evt) & WS_EVT_CONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_CONNECTED);
                ESP_LOGI(TAG, "WS connected, start streaming");
                char start_json[160];
                int n = snprintf(start_json, sizeof(start_json),
                    "{\"type\":\"start\",\"device\":\"%s\",\"wake\":\"hi,lexin\"}",
                    s_device);
                ws_cfg_send_text(start_json);
                tx_n = 0;
                hangover = 0;
                t_stream_start = xTaskGetTickCount();
                s_state = SESS_STREAMING;
            } else if ((xEventGroupGetBits(s_evt) & WS_EVT_DISCONNECTED) ||
                       (xTaskGetTickCount() - t_connect_start > pdMS_TO_TICKS(CONNECT_TIMEOUT_MS))) {
                ESP_LOGW(TAG, "WS connect failed/timeout");
                session_teardown(false);
                ws_cfg_play_tone();
            }
            break;

        /* ---------- 上行：组帧 + VAD ---------- */
        case SESS_STREAMING: {
            for (int i = 0; i < nsamp; i++) {
                tx_frame[tx_n++] = mono[i];
                if (tx_n == PCM_FRAME_SAMPLES) {
                    ws_cfg_send_pcm((uint8_t *)tx_frame, sizeof(tx_frame));
                    tx_n = 0;
                }
            }

            long long sum = 0;
            for (int i = 0; i < nsamp; i++) sum += (long long)mono[i] * mono[i];
            if (sum / nsamp < (long long)VAD_RMS_THRESHOLD * VAD_RMS_THRESHOLD) {
                if (++hangover >= VAD_HANGOVER) goto stream_end;
            } else {
                hangover = 0;
            }

            if (xTaskGetTickCount() - t_stream_start > pdMS_TO_TICKS(STREAM_MAX_MS)) {
                goto stream_end;
            }
            if (xEventGroupGetBits(s_evt) & WS_EVT_DISCONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_DISCONNECTED);
                session_teardown(false);
            }
            break;

stream_end:
            /* 残余帧 + end 帧 → 等 TTS */
            if (tx_n > 0) {
                ws_cfg_send_pcm((uint8_t *)tx_frame, tx_n * 2);
                tx_n = 0;
            }
            int ms = (int)((xTaskGetTickCount() - t_stream_start) * portTICK_PERIOD_MS);
            char end_json[64];
            snprintf(end_json, sizeof(end_json),
                     "{\"type\":\"end\",\"duration_ms\":%d}", ms);
            ws_cfg_send_text(end_json);
            ESP_LOGI(TAG, "Stream end (%d ms), waiting tts", ms);
            s_state = SESS_WAIT_TTS;
            t_wait_start = xTaskGetTickCount();
            break;
        }

        /* ---------- 等 TTS：首字节音频 → 播放；done/超时 → 收尾 ---------- */
        case SESS_WAIT_TTS: {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            if (rb_bytes_filled(s_ring) > 0) {
                ESP_LOGI(TAG, "TTS audio arrived, playing");
                t_play_start = xTaskGetTickCount();
                s_state = SESS_PLAYING;
            } else if (bits & WS_EVT_TTS_DONE) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_DONE);
                ESP_LOGI(TAG, "done without audio");
                session_teardown(true);
            } else if (bits & WS_EVT_TTS_ERROR) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_ERROR);
                session_teardown(false);
            } else if (bits & WS_EVT_DISCONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_DISCONNECTED);
                session_teardown(false);
            } else if (xTaskGetTickCount() - t_wait_start > pdMS_TO_TICKS(WAIT_TTS_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "TTS timeout");
                session_teardown(false);
            }
            break;
        }

        /* ---------- 播放中：done → 排空收尾；异常/超时 → 强制收尾 ---------- */
        case SESS_PLAYING: {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            if (bits & WS_EVT_TTS_DONE) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_DONE);
                ESP_LOGI(TAG, "TTS done, draining");
                session_teardown(true);
            } else if (bits & WS_EVT_TTS_ERROR) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_ERROR);
                session_teardown(false);
            } else if (bits & WS_EVT_DISCONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_DISCONNECTED);
                session_teardown(false);
            } else if (xTaskGetTickCount() - t_play_start > pdMS_TO_TICKS(PLAY_MAX_MS)) {
                ESP_LOGW(TAG, "Play timeout");
                session_teardown(false);
            }
            break;
        }
        }
    }
}

/* ================================================================ */
/*  提示音：播放 voice_data 分区 WAV（16kHz/16bit/mono）             */
/* ================================================================ */

esp_err_t ws_cfg_play_tone(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "voice_data");
    if (part == NULL) {
        ESP_LOGE(TAG, "voice_data partition not found");
        return ESP_FAIL;
    }

    esp_partition_mmap_handle_t map_handle;
    const void *map = NULL;
    if (esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA,
                           &map, &map_handle) != ESP_OK) {
        ESP_LOGE(TAG, "mmap failed");
        return ESP_FAIL;
    }

    const uint8_t *wav = (const uint8_t *)map;
    uint32_t data_off = 0, data_len = 0;
    if (part->size >= 44 && memcmp(wav, "RIFF", 4) == 0 &&
        memcmp(wav + 8, "WAVE", 4) == 0) {
        /* 遍历 chunk 找 data */
        uint32_t off = 12;
        while (off + 8 <= part->size) {
            uint32_t size = wav[off + 4] | (wav[off + 5] << 8) |
                            (wav[off + 6] << 16) | ((uint32_t)wav[off + 7] << 24);
            if (memcmp(wav + off, "data", 4) == 0) {
                data_off = off + 8;
                data_len = size;
                break;
            }
            off += 8 + size + (size & 1);
        }
    }

    if (data_len == 0 || data_off + data_len > part->size) {
        ESP_LOGE(TAG, "Invalid WAV in partition");
        esp_partition_munmap(map_handle);
        return ESP_FAIL;
    }

    /* 分块阻塞播放（板级自动 16bit→32bit、mono→stereo） */
    uint32_t pos = 0;
    while (pos < data_len) {
        uint32_t chunk = data_len - pos > 1024 ? 1024 : data_len - pos;
        esp_audio_play((const int16_t *)(wav + data_off + pos), (int)chunk, portMAX_DELAY);
        pos += chunk;
    }
    esp_partition_munmap(map_handle);
    ESP_LOGI(TAG, "Tone played (%u bytes)", data_len);
    return ESP_OK;
}

/* ================================================================ */
/*  初始化                                                           */
/* ================================================================ */

esp_err_t ws_cfg_init(void)
{
    if (s_evt == NULL) s_evt = xEventGroupCreate();
    if (s_ring == NULL) s_ring = rb_create(RING_BLOCK_SIZE, RING_N_BLOCKS);
    if (s_evt == NULL || s_ring == NULL) {
        ESP_LOGE(TAG, "Event group / ringbuf create failed");
        return ESP_ERR_NO_MEM;
    }
    ws_cfg_attach(s_evt, s_ring);

    /* 设备唯一 ID（与 MQTT client_id 同源） */
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_device, sizeof(s_device),
                 "%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    /* WakeNet 初始化（失败回退"仅播放"模式，不阻塞主流程） */
    s_wn = esp_wn_handle_from_name(WAKE_MODEL_NAME);
    if (s_wn == NULL) {
        ESP_LOGE(TAG, "WakeNet %s not found", WAKE_MODEL_NAME);
    } else {
        s_wn_data = s_wn->create(WAKE_MODEL_NAME, DET_MODE_90);
        if (s_wn_data == NULL) {
            ESP_LOGE(TAG, "WakeNet create failed (OOM?), fallback to play-only");
            s_wn = NULL;
        } else {
            s_wn_chunk = s_wn->get_samp_chunksize(s_wn_data);
            ESP_LOGI(TAG, "WakeNet %s ready, chunk=%d samples, rate=%d",
                     WAKE_MODEL_NAME, s_wn_chunk, s_wn->get_samp_rate(s_wn_data));
        }
    }

    xTaskCreatePinnedToCore(session_task, "session_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(player_task,  "player_task",  4096, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "Voice session initialized (device=%s)", s_device);
    return ESP_OK;
}
