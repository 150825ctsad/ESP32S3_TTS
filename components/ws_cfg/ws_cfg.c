/* ws_cfg.c  -- WebSocket transport
 *
 * 负责 esp_websocket_client 的生命周期与数据分发：
 *   - 上行：voice_session 通过 ws_cfg_send_text / ws_cfg_send_pcm 发送
 *   - 下行：文本帧（JSON 控制）→ 置事件位；二进制帧（PCM）→ 写播放环形缓冲
 *
 * 连接策略：连接生命周期 = 一次语音会话。唤醒时 ws_cfg_connect()，
 * 会话结束 ws_cfg_disconnect()，下次唤醒重新 init+start。
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "ws_cfg.h"
#include "ws_cfg_internal.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "WS_CFG"

#define WS_URI_MAX          256
#define WS_BUFFER_SIZE      4096   /* 对齐云端 4096B PCM 帧 */
#define WS_NET_TIMEOUT_MS   20000
#define WS_TASK_STACK       8192
#define WS_SEND_TIMEOUT_MS  100
#define PLAY_HZ             16000
#define RS_OUT_MAX          4096
#define RS_IN_MAX           8
#define PCM_WRITE_WAIT_MS   2000

static esp_websocket_client_handle_t s_client = NULL;
static char s_uri_dialog[WS_URI_MAX] = {0};
static char s_uri_push[WS_URI_MAX] = {0};
static char s_uri_active[WS_URI_MAX] = {0};
static bool s_screen_enabled = false;
static ws_cfg_text_cb_t s_text_cb = NULL;

static EventGroupHandle_t s_evt = NULL;
static ringbuf_handle_t s_ring = NULL;

/* 云端 header.sampleRate → 板端 16 kHz 线性重采样状态 */
static int s_src_hz = PLAY_HZ;
static int16_t s_rs_in[RS_IN_MAX];
static int s_rs_nin = 0;
static int s_rs_frac = 0;
static uint8_t s_odd_byte;
static bool s_have_odd = false;
static int16_t s_rs_out[RS_OUT_MAX];
static uint8_t s_pcm_align[WS_BUFFER_SIZE + 2] __attribute__((aligned(4)));
static int s_pcm_bytes_in;
static int s_pcm_bytes_out;
static int s_header_bytes;

static void tts_rx_reset(int hz)
{
    s_src_hz = (hz > 0) ? hz : PLAY_HZ;
    s_rs_nin = 0;
    s_rs_frac = 0;
    s_have_odd = false;
    s_pcm_bytes_in = 0;
    s_pcm_bytes_out = 0;
    s_header_bytes = 0;
}

static bool virt_sample(int idx, const int16_t *fresh, int nfresh, int16_t *out)
{
    if (idx < s_rs_nin) {
        *out = s_rs_in[idx];
        return true;
    }
    int j = idx - s_rs_nin;
    if (j >= 0 && j < nfresh) {
        *out = fresh[j];
        return true;
    }
    return false;
}

static int pcm_write_play(const uint8_t *data, int len)
{
    if (s_ring == NULL || data == NULL || len <= 0) {
        return 0;
    }

    const uint8_t *p = data;
    int n = len;
    if (s_have_odd && n > 0) {
        if (n + 1 > (int)sizeof(s_pcm_align)) {
            ESP_LOGW(TAG, "PCM stitch overflow, drop odd byte");
            s_have_odd = false;
        } else {
            s_pcm_align[0] = s_odd_byte;
            memcpy(s_pcm_align + 1, data, (size_t)n);
            p = s_pcm_align;
            n += 1;
            s_have_odd = false;
        }
    }
    if (n & 1) {
        s_odd_byte = p[n - 1];
        s_have_odd = true;
        n--;
    }
    if (n < 2) {
        return 0;
    }

    int nfresh = n / 2;
    bool need_rs = !(s_src_hz == PLAY_HZ && s_rs_nin == 0 && s_rs_frac == 0);
    if (need_rs && p != s_pcm_align) {
        if (n > (int)sizeof(s_pcm_align)) {
            n = (int)sizeof(s_pcm_align) & ~1;
            nfresh = n / 2;
        }
        memcpy(s_pcm_align, p, (size_t)n);
        p = s_pcm_align;
    }
    const int16_t *fresh = (const int16_t *)p;
    s_pcm_bytes_in += nfresh * 2;

    if (!need_rs) {
        int w = rb_write(s_ring, (char *)fresh, nfresh * 2, pdMS_TO_TICKS(PCM_WRITE_WAIT_MS));
        if (w > 0) s_pcm_bytes_out += w;
        if (w < nfresh * 2) {
            ESP_LOGW(TAG, "Ring write short (%d/%d)", w, nfresh * 2);
        }
        return w;
    }

    int total = s_rs_nin + nfresh;
    int idx = 0;
    int nout = 0;
    int written = 0;
    int frac = s_rs_frac;

    while (idx < total) {
        int16_t a, b;
        if (!virt_sample(idx, fresh, nfresh, &a)) break;
        if (!virt_sample(idx + 1, fresh, nfresh, &b)) break;

        if (nout >= RS_OUT_MAX) {
            int w = rb_write(s_ring, (char *)s_rs_out, nout * 2, pdMS_TO_TICKS(PCM_WRITE_WAIT_MS));
            if (w > 0) {
                written += w;
                s_pcm_bytes_out += w;
            }
            if (w < nout * 2) {
                ESP_LOGW(TAG, "Ring write short (%d/%d)", w, nout * 2);
            }
            nout = 0;
        }

        s_rs_out[nout++] = (int16_t)(((int32_t)a * (PLAY_HZ - frac) + (int32_t)b * frac) / PLAY_HZ);
        frac += s_src_hz;
        idx += frac / PLAY_HZ;
        frac %= PLAY_HZ;
    }

    if (nout > 0) {
        int w = rb_write(s_ring, (char *)s_rs_out, nout * 2, pdMS_TO_TICKS(PCM_WRITE_WAIT_MS));
        if (w > 0) {
            written += w;
            s_pcm_bytes_out += w;
        }
        if (w < nout * 2) {
            ESP_LOGW(TAG, "Ring write short (%d/%d)", w, nout * 2);
        }
    }

    int keep_from = idx;
    if (keep_from > total) keep_from = total;
    int nkeep = total - keep_from;
    if (nkeep > RS_IN_MAX) {
        ESP_LOGW(TAG, "Resample leftover overflow (%d), drop", nkeep);
        nkeep = RS_IN_MAX;
        keep_from = total - nkeep;
    }
    int16_t keep[RS_IN_MAX];
    for (int i = 0; i < nkeep; i++) {
        int16_t v = 0;
        virt_sample(keep_from + i, fresh, nfresh, &v);
        keep[i] = v;
    }
    memcpy(s_rs_in, keep, (size_t)nkeep * sizeof(int16_t));
    s_rs_nin = nkeep;
    s_rs_frac = frac;
    return written;
}

/* ================================================================ */
/*  事件回调                                                         */
/* ================================================================ */

static void ws_event_cb(void *arg, esp_event_base_t base,
                        int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected: %s", s_uri_active);
        if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_CONNECTED);
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected");
        if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_DISCONNECTED);
        break;

    case WEBSOCKET_EVENT_ERROR:
        if (data) {
            ESP_LOGE(TAG, "WS error: sock_errno=%d",
                     data->error_handle.esp_transport_sock_errno);
        }
        break;

    case WEBSOCKET_EVENT_DATA: {
        if (data == NULL || data->data_ptr == NULL || data->data_len == 0) {
            /* 控制帧（ping/pong/close）或超大帧分片，payload 为空 */
            break;
        }

        if (data->op_code == WS_TRANSPORT_OPCODES_TEXT) {
            /* 文本帧 = JSON 控制消息 */
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (root == NULL) {
                ESP_LOGW(TAG, "Bad JSON: %.*s", data->data_len, data->data_ptr);
                break;
            }
            cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
            if (cJSON_IsString(type) && type->valuestring) {
                if (strcmp(type->valuestring, "header") == 0) {
                    cJSON *sr = cJSON_GetObjectItemCaseSensitive(root, "sampleRate");
                    cJSON *ch = cJSON_GetObjectItemCaseSensitive(root, "channels");
                    cJSON *bits = cJSON_GetObjectItemCaseSensitive(root, "bits");
                    cJSON *nbytes = cJSON_GetObjectItemCaseSensitive(root, "bytes");
                    int hz = PLAY_HZ;
                    if (cJSON_IsNumber(sr) && sr->valuedouble > 0) {
                        hz = (int)sr->valuedouble;
                    }
                    tts_rx_reset(hz);
                    if (cJSON_IsNumber(nbytes) && nbytes->valuedouble > 0) {
                        s_header_bytes = (int)nbytes->valuedouble;
                    }
                    ESP_LOGI(TAG, "TTS header: %d Hz ch=%d bits=%d bytes=%d -> play %d Hz",
                             s_src_hz,
                             cJSON_IsNumber(ch) ? (int)ch->valuedouble : 1,
                             cJSON_IsNumber(bits) ? (int)bits->valuedouble : 16,
                             cJSON_IsNumber(nbytes) ? (int)nbytes->valuedouble : 0,
                             PLAY_HZ);
                    if (cJSON_IsNumber(ch) && (int)ch->valuedouble != 1) {
                        ESP_LOGW(TAG, "TTS header channels=%d, only mono is played",
                                 (int)ch->valuedouble);
                    }
                } else if (strcmp(type->valuestring, "ready") == 0) {
                    ESP_LOGI(TAG, "Received ready");
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_READY);
                } else if (strcmp(type->valuestring, "listening") == 0) {
                    ESP_LOGI(TAG, "Received listening (server ready for PCM)");
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_LISTENING);
                } else if (strcmp(type->valuestring, "done") == 0 ||
                           strcmp(type->valuestring, "end") == 0) {
                    ESP_LOGI(TAG, "Received %s (pcm in=%d out=%d)",
                             type->valuestring, s_pcm_bytes_in, s_pcm_bytes_out);
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_TTS_DONE);
                } else if (strcmp(type->valuestring, "error") == 0) {
                    cJSON *err = cJSON_GetObjectItemCaseSensitive(root, "error");
                    if (!cJSON_IsString(err) || !err->valuestring) {
                        err = cJSON_GetObjectItemCaseSensitive(root, "message");
                    }
                    if (!cJSON_IsString(err) || !err->valuestring) {
                        err = cJSON_GetObjectItemCaseSensitive(root, "msg");
                    }
                    ESP_LOGW(TAG, "Received error: %s",
                             (cJSON_IsString(err) && err->valuestring)
                                 ? err->valuestring : "(no detail)");
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_TTS_ERROR);
                } else if (strcmp(type->valuestring, "tts_start") == 0) {
                    ESP_LOGI(TAG, "Received tts_start");
                } else if (strcmp(type->valuestring, "text") == 0 ||
                           strcmp(type->valuestring, "transcript") == 0) {
                    cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "text");
                    if (!cJSON_IsString(text) || !text->valuestring) {
                        text = cJSON_GetObjectItemCaseSensitive(root, "transcript");
                    }
                    if (cJSON_IsString(text) && text->valuestring) {
                        ESP_LOGI(TAG, "Recognized text: %s", text->valuestring);
                        if (s_text_cb) s_text_cb(text->valuestring);
                    }
                } else {
                    cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "text");
                    if (cJSON_IsString(text) && text->valuestring) {
                        ESP_LOGI(TAG, "Recognized text: %s", text->valuestring);
                        if (s_text_cb) s_text_cb(text->valuestring);
                    } else {
                        ESP_LOGI(TAG, "Unknown control: %.*s", data->data_len, data->data_ptr);
                    }
                }
            }
            cJSON_Delete(root);
        } else if (data->op_code == WS_TRANSPORT_OPCODES_BINARY ||
                   data->op_code == WS_TRANSPORT_OPCODES_CONT) {
            /* 二进制帧 = PCM → 重采样到 16 kHz 后写入播放缓冲 */
            int n = pcm_write_play((const uint8_t *)data->data_ptr, data->data_len);
            if (n < 0) {
                ESP_LOGW(TAG, "Ring write failed (%d), pcm dropped", n);
            }
        }
        break;
    }

    default:
        break;
    }
}

/* ================================================================ */
/*  公开 API                                                         */
/* ================================================================ */

esp_err_t ws_cfg_set_uri(const char *uri)
{
    if (uri == NULL || strlen(uri) >= WS_URI_MAX) {
        ESP_LOGE(TAG, "Invalid uri");
        return ESP_ERR_INVALID_ARG;
    }
    char tmp[WS_URI_MAX];
    /* 归一化 https:// → wss://，http:// → ws://（mqtt_cfg 拼出 https://） */
    if (strncmp(uri, "https://", 8) == 0) {
        snprintf(tmp, sizeof(tmp), "wss://%s", uri + 8);
    } else if (strncmp(uri, "http://", 7) == 0) {
        snprintf(tmp, sizeof(tmp), "ws://%s", uri + 7);
    } else {
        snprintf(tmp, sizeof(tmp), "%s", uri);
    }
    const char *screen_q = strstr(tmp, "screen=");
    if (screen_q && (screen_q[7] == '0' || screen_q[7] == '1')) {
        s_screen_enabled = (screen_q[7] == '1');
    }
    if (strstr(tmp, "/tcm/") != NULL) {
        snprintf(s_uri_push, sizeof(s_uri_push), "%s", tmp);
        ESP_LOGI(TAG, "WS push uri saved: %s", s_uri_push);
    } else {
        snprintf(s_uri_dialog, sizeof(s_uri_dialog), "%s", tmp);
        ESP_LOGI(TAG, "WS dialog uri saved: %s", s_uri_dialog);
    }
    return ESP_OK;
}

void ws_cfg_set_screen(bool enabled)
{
    s_screen_enabled = enabled;
}

bool ws_cfg_screen_enabled(void)
{
    return s_screen_enabled;
}

esp_err_t ws_cfg_set_default_chat_uri(const char *mac)
{
    if (mac == NULL || mac[0] == '\0') return ESP_ERR_INVALID_ARG;
    char uri[WS_URI_MAX];
    int n = snprintf(uri, sizeof(uri),
                     "ws://iot.gejia.tech/ws/chat/%s?screen=%d",
                     mac, s_screen_enabled ? 1 : 0);
    if (n <= 0 || n >= (int)sizeof(uri)) return ESP_ERR_INVALID_ARG;
    return ws_cfg_set_uri(uri);
}

void ws_cfg_set_text_cb(ws_cfg_text_cb_t cb)
{
    s_text_cb = cb;
}

bool ws_cfg_has_uri(void)
{
    return s_uri_dialog[0] != '\0';
}

bool ws_cfg_has_push_uri(void)
{
    return s_uri_push[0] != '\0';
}

/* ================================================================ */
/*  transport API（voice_session 使用）                              */
/* ================================================================ */

void ws_cfg_attach(EventGroupHandle_t evt, ringbuf_handle_t ring)
{
    s_evt = evt;
    s_ring = ring;
}

esp_err_t ws_cfg_connect(bool push)
{
    if (s_client != NULL) {
        ESP_LOGW(TAG, "Already connecting/connected");
        return ESP_ERR_INVALID_STATE;
    }
    const char *uri = NULL;
    if (push) {
        uri = s_uri_push[0] ? s_uri_push : (s_uri_dialog[0] ? s_uri_dialog : NULL);
    } else {
        uri = s_uri_dialog[0] ? s_uri_dialog : NULL;
    }
    if (uri == NULL) {
        ESP_LOGE(TAG, "No %s uri saved", push ? "push" : "dialog");
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(s_uri_active, sizeof(s_uri_active), "%s", uri);

    const esp_websocket_client_config_t cfg = {
        .uri = s_uri_active,
        .buffer_size = WS_BUFFER_SIZE,
        .network_timeout_ms = WS_NET_TIMEOUT_MS,
        .task_stack = WS_TASK_STACK,
        .disable_auto_reconnect = true,   /* 连接生命周期 = 会话，手动控制 */
        .reconnect_timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,  /* 公网 CA 根证书 bundle */
    };

    tts_rx_reset(PLAY_HZ);
    s_client = esp_websocket_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "WS client init failed");
        return ESP_FAIL;
    }
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY,
                                  ws_event_cb, NULL);
    if (esp_websocket_client_start(s_client) != ESP_OK) {
        ESP_LOGE(TAG, "WS client start failed");
        esp_websocket_client_destroy(s_client);
        s_client = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WS connecting (%s): %s", push ? "push" : "dialog", s_uri_active);
    return ESP_OK;
}

esp_err_t ws_cfg_disconnect(void)
{
    if (s_client == NULL) return ESP_OK;
    /* destroy() 内部会 stop；对已关闭连接再 close+stop 会打
     * "Client was not started" */
    esp_websocket_client_destroy(s_client);
    s_client = NULL;
    tts_rx_reset(PLAY_HZ);
    ESP_LOGI(TAG, "WS closed");
    return ESP_OK;
}

bool ws_cfg_is_connected(void)
{
    return s_client != NULL && esp_websocket_client_is_connected(s_client);
}

esp_err_t ws_cfg_send_text(const char *json)
{
    if (s_client == NULL || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "Not connected, text dropped");
        return ESP_ERR_INVALID_STATE;
    }
    int attempts = 2;
    int r = -1;
    for (int i = 0; i < attempts; i++) {
        r = esp_websocket_client_send_text(s_client, json,
                                           (int)strlen(json),
                                           pdMS_TO_TICKS(WS_SEND_TIMEOUT_MS));
        if (r > 0) break;
        /* short delay before retry */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (r <= 0) {
        ESP_LOGW(TAG, "WS send_text returned %d after retries, closing connection", r);
        /* Treat 0 (no bytes written) as an error and close client so higher-level logic can recover */
        ws_cfg_disconnect();
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ws_cfg_send_pcm(const uint8_t *data, int len)
{
    if (s_client == NULL || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "Not connected, pcm dropped");
        return ESP_ERR_INVALID_STATE;
    }
    int attempts = 2;
    int r = -1;
    for (int i = 0; i < attempts; i++) {
        r = esp_websocket_client_send_bin(s_client, (const char *)data, len,
                                          pdMS_TO_TICKS(WS_SEND_TIMEOUT_MS));
        if (r > 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (r <= 0) {
        ESP_LOGW(TAG, "WS send_bin returned %d after retries, closing connection", r);
        /* Treat 0 (no bytes written) as an error and close client so higher-level logic can recover */
        ws_cfg_disconnect();
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ws_cfg_send_commit(void)
{
    return ws_cfg_send_text("{\"type\":\"commit\"}");
}

esp_err_t ws_cfg_send_bye(void)
{
    return ws_cfg_send_text("{\"type\":\"bye\"}");
}

bool ws_cfg_pcm_complete(void)
{
    if (s_pcm_bytes_in <= 0) {
        return false;
    }
    if (s_header_bytes <= 0) {
        return s_pcm_bytes_in >= 512;
    }
    int slack = s_header_bytes / 50;
    if (slack < 64) slack = 64;
    return (s_pcm_bytes_in + slack) >= s_header_bytes;
}

bool ws_cfg_pcm_had_audio(void)
{
    return s_pcm_bytes_in > 0 || s_pcm_bytes_out > 0;
}

int ws_cfg_pcm_expected_play_bytes(void)
{
    if (s_header_bytes <= 0) {
        return 0;
    }
    int hz = (s_src_hz > 0) ? s_src_hz : PLAY_HZ;
    return (int)((int64_t)s_header_bytes * PLAY_HZ / hz);
}
