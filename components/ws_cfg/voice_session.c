/* voice_session.c  -- 语音会话状态机
 *
 * 参考 xiaozhi-esp32：ESP-SR AFE 双管线
 *   待机：AFE_TYPE_SR + WakeNet（你好小易）
 *   上行：AFE_TYPE_VC + WebRTC NS + VAD（单麦，无参考通道，不开设备 AEC）
 *
 * 状态机：
 *   IDLE ──AFE 唤醒──▶ CONNECTING ──连接成功──▶ STREAMING ──VAD静音/10s──▶ WAIT_TTS
 *     ▲                     │(25s超时: 提示音)                              │(推送15s/对讲5s)
 *     └── done/error ◀── PLAYING ◀── TTS PCM
 *   MQTT 推送：IDLE ──PUSH──▶ CONNECTING ──▶ WAIT_TTS（跳过上行）
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ws_cfg.h"
#include "ws_cfg_internal.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "esp_board_init.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_models.h"
#include "esp_nsn_models.h"
#include "esp_vadn_models.h"
#include "model_path.h"
#include "sdkconfig.h"
#ifndef CONFIG_WS_SCREEN_ENABLED
#define CONFIG_WS_SCREEN_ENABLED 0
#endif
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "ringbuf.h"
#include "TTS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#define TAG "VOICE_SESSION"

#define SESS_EVT_WAKE     (1 << 8)
#define SESS_EVT_VAD_END  (1 << 9)

#define MIC_CH_MAX           4
#define MIC_FRAME_SAMPLES    240
#define PCM_FRAME_SAMPLES    512
#define SR_CHUNK_MAX         2048
#define AFE_FEED_MAX         2048

#define RING_BLOCK_SIZE      1024
#define RING_N_BLOCKS        150     /* 150KB = 4.8s @ 16kHz PCM */ 
#define UP_RING_BLOCKS       16
#define PLAY_HZ              16000
#define PLAY_PREBUF_MS       1000
#define PLAY_PREBUF_BYTES    ((PLAY_HZ * 2 * PLAY_PREBUF_MS) / 1000)
#define PLAY_CHUNK           (RING_BLOCK_SIZE * 2)
#define PLAY_DRAIN_MAX_MS    12000

#define VAD_RMS_THRESHOLD    300
#define VAD_HANGOVER         32

#define CONNECT_TIMEOUT_MS   25000
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

typedef enum {
    AFE_PIPE_OFF = 0,  /* 播放期间停 feed/fetch，避免空环告警 */
    AFE_PIPE_WAKE,
    AFE_PIPE_VC,
} afe_pipe_t;

static EventGroupHandle_t s_evt = NULL;
static ringbuf_handle_t s_ring = NULL;
static ringbuf_handle_t s_up_ring = NULL;
static SemaphoreHandle_t s_afe_mux = NULL;
static SemaphoreHandle_t s_play_mux = NULL;

static const esp_wn_iface_t *s_wn = NULL;
static model_iface_data_t *s_wn_data = NULL;
static int s_wn_chunk = 0;
static char s_wn_name[64] = {0};

static const esp_afe_sr_iface_t *s_afe_wake = NULL;
static esp_afe_sr_data_t *s_afe_wake_data = NULL;
static int s_afe_wake_feed = 0;

static const esp_afe_sr_iface_t *s_afe_vc = NULL;
static esp_afe_sr_data_t *s_afe_vc_data = NULL;
static int s_afe_vc_feed = 0;

static volatile afe_pipe_t s_pipe = AFE_PIPE_WAKE;
static volatile bool s_afe_fetch_en = true;
static volatile int s_afe_chunks_fed = 0;
static volatile sess_state_t s_state = SESS_IDLE;
static char s_device[13] = "000000000000";
static volatile bool s_had_speech = false;

/* Recent captured samples circular buffer for debug dump when AFE reports empty ringbuffer */
static int16_t s_recent_samples[256]; /* stores recent mono samples */
static int s_recent_idx = 0;        /* next write index (wraps) */
static TickType_t s_last_afe_dump = 0; /* last time we dumped samples (rate-limit) */
/* AFE consecutive empty/fail counter to trigger a reset if persistent */
static int s_afe_fail_count = 0; /* incremented when fetch returns empty/ESP_FAIL */
static const int S_AFE_FAIL_RESET_THRESHOLD = 3; /* reset after this many consecutive fails */

static char s_push_msgid[80];
static char s_push_tts[768];
static volatile bool s_push_active = false;
static char s_pending_msgid[80];
static char s_pending_tts[768];
static volatile bool s_push_pending = false;
static SemaphoreHandle_t s_push_mux = NULL;
static ws_cfg_push_done_cb_t s_push_done_cb = NULL;
static volatile bool s_play_run = false;

static void play_enable(bool on)
{
    s_play_run = on;
    if (!on && s_ring) {
        rb_unblock_reader(s_ring);
    }
}

/* ================================================================ */
/*  AFE                                                             */
/* ================================================================ */

static void afe_set_pipe(afe_pipe_t pipe)
{
    if (s_afe_mux) xSemaphoreTake(s_afe_mux, portMAX_DELAY);
    s_pipe = pipe;
    s_afe_chunks_fed = 0;
    if (pipe == AFE_PIPE_WAKE && s_afe_wake && s_afe_wake_data) {
        s_afe_wake->reset_buffer(s_afe_wake_data);
    }
    if (pipe == AFE_PIPE_VC && s_afe_vc && s_afe_vc_data) {
        s_afe_vc->reset_buffer(s_afe_vc_data);
        if (s_afe_vc->reset_vad) s_afe_vc->reset_vad(s_afe_vc_data);
    }
    if (s_afe_mux) xSemaphoreGive(s_afe_mux);
    s_had_speech = false;
}

/* 播放提示音/本地 TTS 时停 fetch，避免 session 阻塞期间空环告警 */
static void afe_pause(void)
{
    s_afe_fetch_en = false;
    if (s_afe_mux) xSemaphoreTake(s_afe_mux, portMAX_DELAY);
    s_pipe = AFE_PIPE_OFF;
    if (s_afe_mux) xSemaphoreGive(s_afe_mux);
    /* 等正在进行的 fetch_with_delay(50ms) 返回后再停喂数侧的告警窗口 */
    vTaskDelay(pdMS_TO_TICKS(60));
}

static void afe_resume(afe_pipe_t pipe)
{
    afe_set_pipe(pipe);
    s_afe_fetch_en = true;
}

static void afe_feed_mono(const int16_t *mono, int nsamp)
{
    static int16_t acc[AFE_FEED_MAX];
    static int acc_n = 0;
    static afe_pipe_t acc_pipe = AFE_PIPE_WAKE;

    const esp_afe_sr_iface_t *iface = NULL;
    esp_afe_sr_data_t *data = NULL;
    int chunk = 0;
    afe_pipe_t pipe = s_pipe;

    if (pipe != acc_pipe) {
        acc_n = 0;
        acc_pipe = pipe;
    }

    if (pipe == AFE_PIPE_WAKE && s_afe_wake && s_afe_wake_data) {
        iface = s_afe_wake;
        data = s_afe_wake_data;
        chunk = s_afe_wake_feed;
    } else if (pipe == AFE_PIPE_VC && s_afe_vc && s_afe_vc_data) {
        iface = s_afe_vc;
        data = s_afe_vc_data;
        chunk = s_afe_vc_feed;
    } else {
        acc_n = 0;
        return;
    }
    if (chunk <= 0 || chunk > AFE_FEED_MAX) return;

    for (int i = 0; i < nsamp; i++) {
        if (acc_n < AFE_FEED_MAX) acc[acc_n++] = mono[i];
        if (acc_n >= chunk) {
            if (s_afe_mux) xSemaphoreTake(s_afe_mux, portMAX_DELAY);
            if (s_pipe == pipe) {
                iface->feed(data, acc);
                s_afe_chunks_fed++;
            }
            if (s_afe_mux) xSemaphoreGive(s_afe_mux);
            acc_n = 0;
        }
    }
}

static bool afe_init_wake(srmodel_list_t *models)
{
    afe_config_t *cfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (cfg == NULL) return false;
    cfg->aec_init = false;
    cfg->se_init = false;
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    cfg->afe_perferred_core = 1;
    cfg->afe_perferred_priority = 1;
    cfg->wakenet_init = true;
    if (s_wn_name[0]) cfg->wakenet_model_name = s_wn_name;
    cfg->wakenet_mode = DET_MODE_90;

    s_afe_wake = esp_afe_handle_from_config(cfg);
    if (s_afe_wake == NULL) {
        afe_config_free(cfg);
        return false;
    }
    s_afe_wake_data = s_afe_wake->create_from_config(cfg);
    afe_config_print(cfg);
    afe_config_free(cfg);
    if (s_afe_wake_data == NULL) {
        s_afe_wake = NULL;
        return false;
    }
    s_afe_wake_feed = s_afe_wake->get_feed_chunksize(s_afe_wake_data);
    s_afe_wake->print_pipeline(s_afe_wake_data);
    ESP_LOGI(TAG, "AFE wake ready, feed=%d", s_afe_wake_feed);
    return true;
}

static bool afe_init_vc(srmodel_list_t *models)
{
    afe_config_t *cfg = afe_config_init("M", models, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    if (cfg == NULL) return false;
    cfg->aec_init = false;
    cfg->se_init = false;
    cfg->wakenet_init = false;
    cfg->agc_init = false;
    cfg->vad_init = true;
    cfg->vad_mode = VAD_MODE_0;
    cfg->vad_min_noise_ms = 400;
    cfg->vad_min_speech_ms = 128;
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    cfg->afe_perferred_core = 1;
    cfg->afe_perferred_priority = 1;

    /* AFE 2.4 把 "WEBRTC" 当 flash 模型名去加载会失败；有 nsnet 才开 NS */
    char *ns_name = models ? esp_srmodel_filter(models, ESP_NSNET_PREFIX, NULL) : NULL;
    if (ns_name) {
        cfg->ns_init = true;
        cfg->ns_model_name = ns_name;
        cfg->afe_ns_mode = AFE_NS_MODE_NET;
    } else {
        cfg->ns_init = false;
        cfg->ns_model_name = NULL;
    }
    char *vad_name = models ? esp_srmodel_filter(models, ESP_VADN_PREFIX, NULL) : NULL;
    if (vad_name) cfg->vad_model_name = vad_name;

    s_afe_vc = esp_afe_handle_from_config(cfg);
    if (s_afe_vc == NULL) {
        afe_config_free(cfg);
        return false;
    }
    s_afe_vc_data = s_afe_vc->create_from_config(cfg);
    afe_config_print(cfg);
    afe_config_free(cfg);
    if (s_afe_vc_data == NULL) {
        s_afe_vc = NULL;
        return false;
    }
    s_afe_vc_feed = s_afe_vc->get_feed_chunksize(s_afe_vc_data);
    s_afe_vc->print_pipeline(s_afe_vc_data);
    ESP_LOGI(TAG, "AFE vc ready, feed=%d (NS+VAD)", s_afe_vc_feed);
    return true;
}

static void afe_fetch_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_afe_fetch_en || s_pipe == AFE_PIPE_OFF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        /* reset 之后先等 feed 填了几块，再 fetch，避免 Ringbuffer empty */
        if (s_afe_chunks_fed < 2) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        afe_pipe_t pipe = s_pipe;
        afe_fetch_result_t *res = NULL;

        if (pipe == AFE_PIPE_WAKE && s_afe_wake && s_afe_wake_data) {
            res = s_afe_wake->fetch_with_delay(s_afe_wake_data, pdMS_TO_TICKS(50));
            if (res && res->ret_value != ESP_FAIL &&
                res->wakeup_state == WAKENET_DETECTED &&
                s_state == SESS_IDLE) {
                ESP_LOGI(TAG, "!!! Wake word detected (AFE) !!!");
                tts_abort();
                if (s_evt) xEventGroupSetBits(s_evt, SESS_EVT_WAKE);
            } else if (!res || res->ret_value == ESP_FAIL) {
                /* AFE reported empty ringbuffer or failure. Rate-limited dump of recent samples for debugging. */
                TickType_t now = xTaskGetTickCount();
                if (s_last_afe_dump == 0 || (now - s_last_afe_dump) >= pdMS_TO_TICKS(2000)) {
                    s_last_afe_dump = now;
                    ESP_LOGW(TAG, "AFE fetch failed (wake). pipe=%d fetch_en=%d chunks_fed=%d state=%d",
                             (int)pipe, (int)s_afe_fetch_en, (int)s_afe_chunks_fed, (int)s_state);
                    /* print last 32 samples */
                    int nprint = 32;
                    char buf[256];
                    int off = 0;
                    off += snprintf(buf + off, sizeof(buf) - off, "recent_samples[");
                    int idx = s_recent_idx - nprint;
                    if (idx < 0) idx += (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                    for (int i = 0; i < nprint; i++) {
                        int ridx = (idx + i) % (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                        off += snprintf(buf + off, sizeof(buf) - off, "%d%s",
                                         s_recent_samples[ridx], (i + 1 < nprint) ? "," : "");
                        if (off >= (int)sizeof(buf) - 32) break;
                    }
                    off += snprintf(buf + off, sizeof(buf) - off, "]");
                    ESP_LOGW(TAG, "%s", buf);
                }
                /* success -> clear fail counter */
                s_afe_fail_count = 0;
            } else if (!res || res->ret_value == ESP_FAIL) {
                /* AFE reported empty ringbuffer or failure. Rate-limited dump of recent samples for debugging. */
                TickType_t now = xTaskGetTickCount();
                if (s_last_afe_dump == 0 || (now - s_last_afe_dump) >= pdMS_TO_TICKS(2000)) {
                    s_last_afe_dump = now;
                    ESP_LOGW(TAG, "AFE fetch failed (wake). pipe=%d fetch_en=%d chunks_fed=%d state=%d",
                             (int)pipe, (int)s_afe_fetch_en, (int)s_afe_chunks_fed, (int)s_state);
                    /* print last 32 samples */
                    int nprint = 32;
                    char buf[256];
                    int off = 0;
                    off += snprintf(buf + off, sizeof(buf) - off, "recent_samples[");
                    int idx = s_recent_idx - nprint;
                    if (idx < 0) idx += (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                    for (int i = 0; i < nprint; i++) {
                        int ridx = (idx + i) % (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                        off += snprintf(buf + off, sizeof(buf) - off, "%d%s",
                                         s_recent_samples[ridx], (i + 1 < nprint) ? "," : "");
                        if (off >= (int)sizeof(buf) - 32) break;
                    }
                    off += snprintf(buf + off, sizeof(buf) - off, "]");
                    ESP_LOGW(TAG, "%s", buf);
                }
                /* increment fail counter and attempt reset when persistent */
                s_afe_fail_count++;
                if (s_afe_fail_count >= S_AFE_FAIL_RESET_THRESHOLD) {
                    ESP_LOGW(TAG, "AFE persistent empty: resetting AFE wake buffer (fail_count=%d)", s_afe_fail_count);
                    if (s_afe_wake && s_afe_wake_data && s_afe_wake->reset_buffer) {
                        s_afe_wake->reset_buffer(s_afe_wake_data);
                    }
                    s_afe_chunks_fed = 0;
                    s_afe_fail_count = 0;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        } else if (pipe == AFE_PIPE_VC && s_afe_vc && s_afe_vc_data) {
            res = s_afe_vc->fetch_with_delay(s_afe_vc_data, pdMS_TO_TICKS(50));
            if (res && res->ret_value != ESP_FAIL && res->data && res->data_size > 0) {
                if (s_state == SESS_STREAMING && s_up_ring) {
                    rb_write(s_up_ring, (char *)res->data, res->data_size, 0);
                }
                if (s_state == SESS_STREAMING) {
                    if (res->vad_state == VAD_SPEECH) {
                        s_had_speech = true;
                    } else if (res->vad_state == VAD_SILENCE && s_had_speech) {
                        if (s_evt) xEventGroupSetBits(s_evt, SESS_EVT_VAD_END);
                    }
                }
                /* success -> clear fail counter */
                s_afe_fail_count = 0;
            } else if (!res || res->ret_value == ESP_FAIL) {
                TickType_t now = xTaskGetTickCount();
                if (s_last_afe_dump == 0 || (now - s_last_afe_dump) >= pdMS_TO_TICKS(2000)) {
                    s_last_afe_dump = now;
                    ESP_LOGW(TAG, "AFE fetch failed (vc). pipe=%d fetch_en=%d chunks_fed=%d state=%d",
                             (int)pipe, (int)s_afe_fetch_en, (int)s_afe_chunks_fed, (int)s_state);
                    int nprint = 32;
                    char buf[256];
                    int off = 0;
                    off += snprintf(buf + off, sizeof(buf) - off, "recent_samples[");
                    int idx = s_recent_idx - nprint;
                    if (idx < 0) idx += (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                    for (int i = 0; i < nprint; i++) {
                        int ridx = (idx + i) % (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]));
                        off += snprintf(buf + off, sizeof(buf) - off, "%d%s",
                                         s_recent_samples[ridx], (i + 1 < nprint) ? "," : "");
                        if (off >= (int)sizeof(buf) - 32) break;
                    }
                    off += snprintf(buf + off, sizeof(buf) - off, "]");
                    ESP_LOGW(TAG, "%s", buf);
                }
                /* increment fail counter and attempt reset when persistent */
                s_afe_fail_count++;
                if (s_afe_fail_count >= S_AFE_FAIL_RESET_THRESHOLD) {
                    ESP_LOGW(TAG, "AFE persistent empty: resetting AFE vc buffer (fail_count=%d)", s_afe_fail_count);
                    if (s_afe_vc && s_afe_vc_data && s_afe_vc->reset_buffer) {
                        s_afe_vc->reset_buffer(s_afe_vc_data);
                    }
                    s_afe_chunks_fed = 0;
                    s_afe_fail_count = 0;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

/* ================================================================ */
/*  会话收尾                                                         */
/* ================================================================ */

static void play_local_tts_fallback(const char *text, bool *ok)
{
    if (text == NULL || text[0] == '\0') {
        return;
    }
    if (!tts_ready()) {
        ESP_LOGW(TAG, "Local TTS unavailable, skip fallback");
        return;
    }
    ESP_LOGI(TAG, "Fallback local TTS: %s", text);
    afe_pause();
    if (tts_play(text) == ESP_OK) {
        *ok = true;
    }
    afe_resume(AFE_PIPE_WAKE);
}

static bool ws_preferred(void)
{
    return ws_cfg_has_push_uri() || ws_cfg_has_uri();
}

static void push_promote_pending(void)
{
    if (!s_push_pending) {
        return;
    }
    snprintf(s_push_msgid, sizeof(s_push_msgid), "%s", s_pending_msgid);
    snprintf(s_push_tts, sizeof(s_push_tts), "%s", s_pending_tts);
    s_pending_msgid[0] = '\0';
    s_pending_tts[0] = '\0';
    s_push_pending = false;
    if (s_evt) {
        xEventGroupSetBits(s_evt, SESS_EVT_PUSH);
    }
    ESP_LOGI(TAG, "Pending push promoted (msgId=%s)",
             s_push_msgid[0] ? s_push_msgid : "-");
}

static void session_teardown(bool normal)
{
    char msgid[sizeof(s_push_msgid)];
    char tts[sizeof(s_push_tts)];
    bool was_push = s_push_active;
    msgid[0] = '\0';
    tts[0] = '\0';
    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
    if (was_push) {
        snprintf(msgid, sizeof(msgid), "%s", s_push_msgid);
        snprintf(tts, sizeof(tts), "%s", s_push_tts);
        s_push_active = false;
        s_push_msgid[0] = '\0';
        s_push_tts[0] = '\0';
    }
    if (s_push_mux) xSemaphoreGive(s_push_mux);

    if (normal) {
        rb_done_write(s_ring);
        play_enable(true);
        ESP_LOGI(TAG, "Draining playback");
        TickType_t t0 = xTaskGetTickCount();
        while (s_ring && rb_bytes_filled(s_ring) > 0 &&
               (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(PLAY_DRAIN_MAX_MS)) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(80));
        play_enable(false);
    } else {
        play_enable(false);
        ESP_LOGW(TAG, "Abort playback");
    }
    /* 单句模式：当前句播放完成后显式结束整个 WebSocket 会话。 */
    if (ws_cfg_is_connected()) {
        ws_cfg_send_bye();
    }
    ws_cfg_disconnect();
    afe_resume(AFE_PIPE_WAKE);
    if (s_evt) {
        xEventGroupClearBits(s_evt, SESS_EVT_WAKE | SESS_EVT_VAD_END);
    }
    s_state = SESS_IDLE;
    ESP_LOGI(TAG, "Session teardown -> IDLE");

    bool ok = normal;
    bool skip_local_tts = false;
    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
    if (s_push_pending && ws_preferred()) {
        skip_local_tts = true;
    }
    if (s_push_mux) xSemaphoreGive(s_push_mux);
    /* 云端已经出声：不再叠本地 TTS。无音频才兜底。 */
    if (was_push && !normal && !skip_local_tts && !ws_cfg_pcm_had_audio()) {
        play_local_tts_fallback(tts, &ok);
    }
    if (was_push && msgid[0] && s_push_done_cb) {
        s_push_done_cb(msgid, ok, tts);
    }

    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
    push_promote_pending();
    if (s_push_mux) xSemaphoreGive(s_push_mux);
}

static void start_cloud_session(TickType_t *t_connect_start, bool push)
{
    tts_abort();
    play_enable(false);
    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
    s_push_active = push;
    if (s_push_mux) xSemaphoreGive(s_push_mux);
    xEventGroupClearBits(s_evt,
        WS_EVT_CONNECTED | WS_EVT_DISCONNECTED |
        WS_EVT_TTS_DONE | WS_EVT_TTS_ERROR |
        SESS_EVT_WAKE | SESS_EVT_VAD_END);
    if (push) {
        xEventGroupClearBits(s_evt, SESS_EVT_PUSH);
    }
    afe_pause();
    if (s_up_ring) rb_reset(s_up_ring);
    if (ws_cfg_connect(push) == ESP_OK) {
        s_state = SESS_CONNECTING;
        *t_connect_start = xTaskGetTickCount();
    } else {
        afe_resume(AFE_PIPE_WAKE);
        if (push) {
            session_teardown(false);
        } else {
            if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
            s_push_active = false;
            if (s_push_mux) xSemaphoreGive(s_push_mux);
            ws_cfg_play_tone();
        }
    }
}

static void on_wake_word(TickType_t *t_connect_start)
{
    ESP_LOGI(TAG, "Wake word: ack then dialog WS");
    afe_pause();
    if (tts_ready()) {
        tts_play("在");
    }
    if (!ws_cfg_has_uri()) {
        ESP_LOGW(TAG, "No dialog ws uri yet, tone and ignore");
        afe_resume(AFE_PIPE_WAKE);
        ws_cfg_play_tone();
        return;
    }
    start_cloud_session(t_connect_start, false);
}

void ws_cfg_set_push_done_cb(ws_cfg_push_done_cb_t cb)
{
    s_push_done_cb = cb;
}

esp_err_t ws_cfg_request_push(const char *msg_id, const char *tts_text)
{
    if (s_evt == NULL) {
        ESP_LOGE(TAG, "Voice session not ready");
        return ESP_ERR_INVALID_STATE;
    }
    if (!ws_cfg_has_push_uri() && !ws_cfg_has_uri() &&
        (tts_text == NULL || tts_text[0] == '\0')) {
        ESP_LOGE(TAG, "No ws uri and no tts text for push");
        return ESP_ERR_INVALID_STATE;
    }

    tts_abort();
    play_enable(false);
    xEventGroupClearBits(s_evt,
        WS_EVT_CONNECTED | WS_EVT_DISCONNECTED |
        WS_EVT_TTS_DONE | WS_EVT_TTS_ERROR |
        SESS_EVT_WAKE | SESS_EVT_VAD_END);
    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
    bool busy = s_push_active ||
                (s_evt && (xEventGroupGetBits(s_evt) & SESS_EVT_PUSH));
    if (busy) {
        if (s_push_pending) {
            if (s_push_mux) xSemaphoreGive(s_push_mux);
            ESP_LOGW(TAG, "Push busy, drop (msgId=%s)", msg_id ? msg_id : "-");
            return ESP_ERR_INVALID_STATE;
        }
        snprintf(s_pending_msgid, sizeof(s_pending_msgid), "%s", msg_id ? msg_id : "");
        snprintf(s_pending_tts, sizeof(s_pending_tts), "%s", tts_text ? tts_text : "");
        s_push_pending = true;
        if (s_push_mux) xSemaphoreGive(s_push_mux);
        ESP_LOGI(TAG, "Push TTS deferred (msgId=%s)",
                 s_pending_msgid[0] ? s_pending_msgid : "-");
        return ESP_OK;
    }
    snprintf(s_push_msgid, sizeof(s_push_msgid), "%s", msg_id ? msg_id : "");
    snprintf(s_push_tts, sizeof(s_push_tts), "%s", tts_text ? tts_text : "");
    xEventGroupSetBits(s_evt, SESS_EVT_PUSH);
    if (s_push_mux) xSemaphoreGive(s_push_mux);
    ESP_LOGI(TAG, "Push TTS queued (msgId=%s)", s_push_msgid[0] ? s_push_msgid : "-");
    return ESP_OK;
}

/* ================================================================ */
/*  player_task                                                      */
/* ================================================================ */

static void player_task(void *arg)
{
    (void)arg;
    char buf[PLAY_CHUNK];
    bool underrun_logged = false;
    for (;;) {
        if (!s_play_run) {
            vTaskDelay(pdMS_TO_TICKS(10));
            underrun_logged = false;
            continue;
        }
        /* 等环形缓冲，不灌静音；I2S DMA auto_clear 会自己出零 */
        int n = rb_read(s_ring, buf, sizeof(buf), pdMS_TO_TICKS(250));
        if (n <= 0) {
            if (!s_play_run) {
                rb_reset(s_ring);
                continue;
            }
            if (!underrun_logged) {
                ESP_LOGW(TAG, "Play wait for more PCM");
                underrun_logged = true;
            }
            continue;
        }
        underrun_logged = false;
        esp_audio_play((int16_t *)buf, n, portMAX_DELAY);
    }
}

/* ================================================================ */
/*  session_task                                                     */
/* ================================================================ */

static bool stream_send_commit(int16_t *tx_frame, int *tx_n, TickType_t t_stream_start)
{
    if (*tx_n > 0) {
        if (ws_cfg_send_pcm((uint8_t *)tx_frame, *tx_n * 2) != ESP_OK) {
            ESP_LOGW(TAG, "stream_send_commit: ws_cfg_send_pcm failed");
            *tx_n = 0;
            return false;
        }
        *tx_n = 0;
    }
    int ms = (int)((xTaskGetTickCount() - t_stream_start) * portTICK_PERIOD_MS);
    if (ws_cfg_send_commit() != ESP_OK) {
        ESP_LOGW(TAG, "stream_send_commit: ws_cfg_send_commit failed");
        return false;
    }
    ESP_LOGI(TAG, "Stream commit (%d ms), waiting tts", ms);
    return true;
}

static bool drain_uplink_ring(void)
{
    if (!s_up_ring) return true;
    uint8_t buf[PCM_FRAME_SAMPLES * 2];
    for (;;) {
        int n = rb_read(s_up_ring, (char *)buf, sizeof(buf), 0);
        if (n <= 0) break;
        if (ws_cfg_send_pcm(buf, n) != ESP_OK) return false;
    }
    return true;
}

static void session_task(void *arg)
{
    (void)arg;
    static int16_t frame[MIC_FRAME_SAMPLES * MIC_CH_MAX];
    static int16_t mono[512];
    static int16_t wn_buf[SR_CHUNK_MAX];
    int wn_n = 0;
    int16_t tx_frame[PCM_FRAME_SAMPLES];
    int tx_n = 0;
    int hangover = 0;
    TickType_t t_connect_start = 0, t_stream_start = 0, t_wait_start = 0, t_play_start = 0;
    const bool use_afe_wake = (s_afe_wake && s_afe_wake_data);
    const bool use_afe_vc = (s_afe_vc && s_afe_vc_data);

    for (;;) {
        int ch = esp_get_feed_channel();
        if (ch < 1) ch = 1;
        if (ch > MIC_CH_MAX) ch = MIC_CH_MAX;
        int nbytes = MIC_FRAME_SAMPLES * ch * (int)sizeof(int16_t);
        if (esp_get_feed_data(false, frame, nbytes) != ESP_OK) {
            static TickType_t t_fail;
            TickType_t now = xTaskGetTickCount();
            if (t_fail == 0 || (now - t_fail) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "mic read fail");
                t_fail = now;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int nsamp = MIC_FRAME_SAMPLES;
        if (nsamp > (int)(sizeof(mono) / sizeof(mono[0]))) nsamp = (int)(sizeof(mono) / sizeof(mono[0]));
        for (int i = 0; i < nsamp; i++) mono[i] = frame[ch * i];

        /* 每秒打印一次麦克风能量统计，并输出前几个样本值以便调试 */
        {
            static uint32_t abs_acc = 0, peak = 0, frames = 0;
            static TickType_t t_log = 0;
            uint32_t frame_peak = 0;
            uint32_t abs_sum = 0;
            for (int i = 0; i < nsamp; i++) {
                int v = mono[i];
                uint32_t a = (uint32_t)(v < 0 ? -v : v);
                abs_sum += a;
                if (a > frame_peak) frame_peak = a;
            }
            /* 防止除以 0 */
            if (nsamp > 0) {
                abs_acc += abs_sum / (uint32_t)nsamp;
            }
            if (frame_peak > peak) peak = frame_peak;
            frames++;
            TickType_t now = xTaskGetTickCount();
            if (t_log == 0 || (now - t_log) >= pdMS_TO_TICKS(1000)) {
                /* 构造前几个样本的简短字符串（最多 8 个样本） */
                int samples_to_print = nsamp < 8 ? nsamp : 8;
                char sample_buf[128];
                int off = 0;
                off += snprintf(sample_buf + off, sizeof(sample_buf) - off, "samples[");
                for (int si = 0; si < samples_to_print; si++) {
                    off += snprintf(sample_buf + off, sizeof(sample_buf) - off, "%d%s",
                                     mono[si], (si + 1 < samples_to_print) ? ", " : "");
                }
                off += snprintf(sample_buf + off, sizeof(sample_buf) - off, "]");

                ESP_LOGI(TAG, "mic |avg|=%u peak=%u frames=%u %s",
                         (unsigned)(frames ? abs_acc / frames : 0),
                         (unsigned)peak, (unsigned)frames, sample_buf);
                abs_acc = 0;
                peak = 0;
                frames = 0;
                t_log = now;
            }
        }

        if (use_afe_wake || use_afe_vc) {
            /* Append mono samples to recent circular buffer for debug dumps */
            for (int i = 0; i < nsamp; i++) {
                s_recent_samples[s_recent_idx++] = mono[i];
                if (s_recent_idx >= (int)(sizeof(s_recent_samples)/sizeof(s_recent_samples[0]))) s_recent_idx = 0;
            }
            afe_feed_mono(mono, nsamp);
        }

        switch (s_state) {

        case SESS_IDLE:
            if (xEventGroupGetBits(s_evt) & SESS_EVT_PUSH) {
                xEventGroupClearBits(s_evt, SESS_EVT_PUSH);
                bool use_cloud = ws_preferred();
                if (!use_cloud) {
                    ESP_LOGW(TAG, "No WS uri, local TTS (msgId=%s)",
                             s_push_msgid[0] ? s_push_msgid : "-");
                    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
                    s_push_active = true;
                    if (s_push_mux) xSemaphoreGive(s_push_mux);
                    bool ok = false;
                    play_local_tts_fallback(s_push_tts, &ok);
                    char msgid[sizeof(s_push_msgid)];
                    char tts[sizeof(s_push_tts)];
                    snprintf(msgid, sizeof(msgid), "%s", s_push_msgid);
                    snprintf(tts, sizeof(tts), "%s", s_push_tts);
                    if (s_push_mux) xSemaphoreTake(s_push_mux, portMAX_DELAY);
                    s_push_active = false;
                    s_push_msgid[0] = '\0';
                    s_push_tts[0] = '\0';
                    push_promote_pending();
                    if (s_push_mux) xSemaphoreGive(s_push_mux);
                    if (msgid[0] && s_push_done_cb) {
                        s_push_done_cb(msgid, ok, tts);
                    }
                    break;
                }
                ESP_LOGI(TAG, "Cloud push (WS first): connect WS (msgId=%s)",
                         s_push_msgid[0] ? s_push_msgid : "-");
                start_cloud_session(&t_connect_start, true);
                break;
            }
            if (use_afe_wake) {
                if (xEventGroupGetBits(s_evt) & SESS_EVT_WAKE) {
                    xEventGroupClearBits(s_evt, SESS_EVT_WAKE);
                    on_wake_word(&t_connect_start);
                }
            } else if (s_wn && s_wn_data) {
                for (int i = 0; i < nsamp && wn_n < (int)(sizeof(wn_buf) / sizeof(wn_buf[0])); i++) {
                    wn_buf[wn_n++] = mono[i];
                }
                if (s_wn_chunk > 0 && wn_n >= s_wn_chunk) {
                    int leftover = wn_n - s_wn_chunk;
                    wakenet_state_t st = s_wn->detect(s_wn_data, wn_buf);
                    if (leftover > 0) {
                        memmove(wn_buf, wn_buf + s_wn_chunk, leftover * sizeof(int16_t));
                    }
                    wn_n = leftover;
                    if (st == WAKENET_DETECTED) {
                        ESP_LOGI(TAG, "!!! Wake word detected (raw WN) !!!");
                        wn_n = 0;
                        on_wake_word(&t_connect_start);
                    }
                }
            }
            break;

        case SESS_CONNECTING:
            if (xEventGroupGetBits(s_evt) & WS_EVT_CONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_CONNECTED);
                if (s_push_active) {
                    cJSON *start = cJSON_CreateObject();
                    if (start) {
                        cJSON_AddStringToObject(start, "type", "start");
                        cJSON_AddStringToObject(start, "device", s_device);
                        if (s_push_msgid[0]) {
                            cJSON_AddStringToObject(start, "msgId", s_push_msgid);
                        }
                        if (s_push_tts[0]) {
                            cJSON_AddStringToObject(start, "tts", s_push_tts);
                        }
                        cJSON_AddTrueToObject(start, "cloud");
                        char *js = cJSON_PrintUnformatted(start);
                        if (js) {
                            ws_cfg_send_text(js);
                            cJSON_free(js);
                        }
                        cJSON_Delete(start);
                    }
                    ESP_LOGI(TAG, "WS connected, wait cloud TTS");
                    t_wait_start = xTaskGetTickCount();
                    s_state = SESS_WAIT_TTS;
                    break;
                }
                ESP_LOGI(TAG, "WS connected, start streaming");
                if (s_afe_vc && s_afe_vc_data) {
                    afe_resume(AFE_PIPE_VC);
                } else {
                    afe_resume(AFE_PIPE_WAKE);
                }
                const char *screen = ws_cfg_screen_enabled() ? "true" : "false";
                char start_json[192];
                snprintf(start_json, sizeof(start_json),
                    "{\"type\":\"start\",\"format\":\"pcm\",\"codec\":\"pcm_s16le\",\"sampleRate\":16000,\"channels\":1,\"bits\":16,\"screen\":%s}",
                    screen);
                ws_cfg_send_text(start_json);
                tx_n = 0;
                hangover = 0;
                s_had_speech = false;
                t_stream_start = xTaskGetTickCount();
                s_state = SESS_STREAMING;
            } else if ((xEventGroupGetBits(s_evt) & WS_EVT_DISCONNECTED) ||
                       (xTaskGetTickCount() - t_connect_start > pdMS_TO_TICKS(CONNECT_TIMEOUT_MS))) {
                ESP_LOGW(TAG, "WS connect failed/timeout");
                bool push = s_push_active;
                session_teardown(false);
                if (!push) ws_cfg_play_tone();
            }
            break;

        case SESS_STREAMING: {
            if (use_afe_vc && s_up_ring) {
                char up[PCM_FRAME_SAMPLES * 2];
                int n = rb_read(s_up_ring, up, sizeof(up), 0);
                if (n > 0) {
                    if (ws_cfg_send_pcm((uint8_t *)up, n) != ESP_OK) {
                        session_teardown(false);
                        break;
                    }
                }
            } else {
                for (int i = 0; i < nsamp; i++) {
                    tx_frame[tx_n++] = mono[i];
                    if (tx_n == PCM_FRAME_SAMPLES) {
                        if (ws_cfg_send_pcm((uint8_t *)tx_frame, sizeof(tx_frame)) != ESP_OK) {
                            session_teardown(false);
                            break;
                        }
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
            }

            if (use_afe_vc && (xEventGroupGetBits(s_evt) & SESS_EVT_VAD_END)) {
                xEventGroupClearBits(s_evt, SESS_EVT_VAD_END);
                goto stream_end;
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
            /* 先切换状态，阻止 fetch 任务继续向上行环形缓冲写入。 */
            s_state = SESS_WAIT_TTS;
            afe_pause();
            if (!drain_uplink_ring()) {
                session_teardown(false);
                break;
            }
            if (!stream_send_commit(tx_frame, &tx_n, t_stream_start)) {
                session_teardown(false);
                break;
            }
            t_wait_start = xTaskGetTickCount();
            break;
        }

        case SESS_WAIT_TTS: {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            int filled = rb_bytes_filled(s_ring);
            bool ended = (bits & WS_EVT_TTS_DONE) != 0;
            int expect = ws_cfg_pcm_expected_play_bytes();
            int ring_cap = s_ring ? rb_get_size(s_ring) : 0;
            bool hold_all = (expect > 0 && expect + 4096 < ring_cap);
            bool ready = ended && filled > 0;
            if (!hold_all) {
                ready = ready || (filled >= PLAY_PREBUF_BYTES);
            }
            if (ready) {
                ESP_LOGI(TAG, "TTS play start (buf=%d expect=%d%s)",
                         filled, expect, ended ? ", complete" : ", prebuf");
                play_enable(true);
                t_play_start = xTaskGetTickCount();
                s_state = SESS_PLAYING;
                if (ended) {
                    xEventGroupClearBits(s_evt, WS_EVT_TTS_DONE);
                    if (!ws_cfg_pcm_complete()) {
                        ESP_LOGW(TAG, "PCM incomplete, local TTS fallback");
                        session_teardown(false);
                    } else {
                        ESP_LOGI(TAG, "TTS done, draining");
                        session_teardown(true);
                    }
                }
                break;
            }
            if (bits & WS_EVT_TTS_DONE) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_DONE);
                ESP_LOGI(TAG, "done without audio");
                session_teardown(false);
            } else if (bits & WS_EVT_TTS_ERROR) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_ERROR);
                session_teardown(false);
            } else if (bits & WS_EVT_DISCONNECTED) {
                xEventGroupClearBits(s_evt, WS_EVT_DISCONNECTED);
                session_teardown(false);
            } else if (xTaskGetTickCount() - t_wait_start >
                       pdMS_TO_TICKS(s_push_active ? 25000 : WAIT_TTS_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "TTS timeout");
                session_teardown(false);
            }
            break;
        }

        case SESS_PLAYING: {
            EventBits_t bits = xEventGroupGetBits(s_evt);
            if (bits & WS_EVT_TTS_DONE) {
                xEventGroupClearBits(s_evt, WS_EVT_TTS_DONE);
                if (!ws_cfg_pcm_complete()) {
                    ESP_LOGW(TAG, "PCM incomplete, local TTS fallback");
                    session_teardown(false);
                } else {
                    ESP_LOGI(TAG, "TTS done, draining");
                    session_teardown(true);
                }
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
/*  提示音                                                           */
/* ================================================================ */

static esp_err_t play_wav_buf(const uint8_t *wav, size_t wav_size)
{
    uint32_t data_off = 0, data_len = 0;
    if (wav == NULL || wav_size < 44 || memcmp(wav, "RIFF", 4) != 0 ||
        memcmp(wav + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t off = 12;
    while (off + 8 <= wav_size) {
        uint32_t size = wav[off + 4] | (wav[off + 5] << 8) |
                        (wav[off + 6] << 16) | ((uint32_t)wav[off + 7] << 24);
        if (memcmp(wav + off, "data", 4) == 0) {
            data_off = off + 8;
            data_len = size;
            break;
        }
        off += 8 + size + (size & 1);
    }

    if (data_len == 0 || data_off + data_len > wav_size) {
        ESP_LOGE(TAG, "Invalid WAV data chunk");
        return ESP_FAIL;
    }

    uint32_t pos = 0;
    int16_t ram[512];
    while (pos < data_len) {
        uint32_t chunk = data_len - pos > sizeof(ram) ? sizeof(ram) : data_len - pos;
        memcpy(ram, wav + data_off + pos, chunk);
        /* 只读区（mmap/嵌入二进制）：EQ/软音量会就地改写，必须先拷到 RAM */
        esp_audio_play(ram, (int)chunk, portMAX_DELAY);
        pos += chunk;
    }
    ESP_LOGI(TAG, "WAV played (%u bytes)", (unsigned)data_len);
    return ESP_OK;
}

esp_err_t ws_cfg_play_wav(const uint8_t *wav, size_t wav_size)
{
    if (s_play_mux) xSemaphoreTake(s_play_mux, portMAX_DELAY);
    afe_pipe_t prev = s_pipe;
    afe_pause();
    esp_err_t ret = play_wav_buf(wav, wav_size);
    afe_resume(prev == AFE_PIPE_OFF ? AFE_PIPE_WAKE : prev);
    if (s_play_mux) xSemaphoreGive(s_play_mux);
    return ret;
}

esp_err_t ws_cfg_play_tone(void)
{
    if (s_play_mux) xSemaphoreTake(s_play_mux, portMAX_DELAY);
    afe_pipe_t prev = s_pipe;
    afe_pause();
    esp_err_t ret = ESP_FAIL;

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "voice_data");
    if (part == NULL) {
        ESP_LOGE(TAG, "voice_data partition not found");
        goto out;
    }

    esp_partition_mmap_handle_t map_handle;
    const void *map = NULL;
    if (esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA,
                           &map, &map_handle) != ESP_OK) {
        ESP_LOGE(TAG, "mmap failed");
        goto out;
    }

    ret = play_wav_buf((const uint8_t *)map, part->size);
    esp_partition_munmap(map_handle);

out:
    afe_resume(prev == AFE_PIPE_OFF ? AFE_PIPE_WAKE : prev);
    if (s_play_mux) xSemaphoreGive(s_play_mux);
    return ret;
}

/* ================================================================ */
/*  初始化                                                           */
/* ================================================================ */

esp_err_t ws_cfg_init(void)
{
    ws_cfg_set_screen(CONFIG_WS_SCREEN_ENABLED != 0);
    if (s_evt == NULL) s_evt = xEventGroupCreate();
    if (s_ring == NULL) s_ring = rb_create(RING_BLOCK_SIZE, RING_N_BLOCKS);
    ESP_LOGI(TAG, "Play cache %d KB, prebuf %d ms",
             RING_N_BLOCKS * RING_BLOCK_SIZE / 1024, PLAY_PREBUF_MS);
    if (s_up_ring == NULL) s_up_ring = rb_create(RING_BLOCK_SIZE, UP_RING_BLOCKS);
    if (s_afe_mux == NULL) s_afe_mux = xSemaphoreCreateMutex();
    if (s_play_mux == NULL) s_play_mux = xSemaphoreCreateMutex();
    if (s_push_mux == NULL) s_push_mux = xSemaphoreCreateMutex();
    if (s_evt == NULL || s_ring == NULL || s_up_ring == NULL || s_afe_mux == NULL ||
        s_play_mux == NULL || s_push_mux == NULL) {
        ESP_LOGE(TAG, "Event group / ringbuf / mutex create failed");
        return ESP_ERR_NO_MEM;
    }
    ws_cfg_attach(s_evt, s_ring);

    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_device, sizeof(s_device),
                 "%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        /* MQTT 下发地址可以覆盖默认地址；没有下发时直接连接云端对讲路径。 */
        ws_cfg_set_default_chat_uri(s_device);
    }

    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL || models->num == 0) {
        ESP_LOGE(TAG, "esp_srmodel_init(model) failed, check model partition flash");
    } else {
        ESP_LOGI(TAG, "SR models in /srmodel: %d", models->num);
        for (int i = 0; i < models->num; i++) {
            ESP_LOGI(TAG, "  [%d] %s", i, models->model_name[i] ? models->model_name[i] : "?");
        }
        char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "nihaoxiaoyi");
        if (wn_name == NULL) wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
        if (wn_name) snprintf(s_wn_name, sizeof(s_wn_name), "%s", wn_name);
    }

    ESP_LOGI(TAG, "heap before AFE: intern=%u spiram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (s_wn_name[0]) {
        if (!afe_init_wake(models)) {
            ESP_LOGW(TAG, "AFE wake init failed, fallback raw WakeNet");
        }
    }

    if (!afe_init_vc(models)) {
        ESP_LOGW(TAG, "AFE vc init failed, uplink will be raw PCM + RMS VAD");
    }

    if (s_afe_wake == NULL && s_wn_name[0]) {
        s_wn = esp_wn_handle_from_name(s_wn_name);
        if (s_wn) {
            s_wn_data = s_wn->create(s_wn_name, DET_MODE_90);
            if (s_wn_data) {
                s_wn_chunk = s_wn->get_samp_chunksize(s_wn_data);
                ESP_LOGI(TAG, "Raw WakeNet %s ready, chunk=%d", s_wn_name, s_wn_chunk);
            } else {
                s_wn = NULL;
            }
        }
    }

    afe_set_pipe(AFE_PIPE_WAKE);

    if (tts_init() != ESP_OK) {
        ESP_LOGW(TAG, "Local TTS init failed, cloud-only playback");
    }

    xTaskCreatePinnedToCore(session_task, "session_task", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(afe_fetch_task, "afe_fetch", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(player_task, "player_task", 6144, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "Voice session initialized (device=%s afe_wake=%d afe_vc=%d tts=%d)",
             s_device, s_afe_wake != NULL, s_afe_vc != NULL, tts_ready());
    return ESP_OK;
}
