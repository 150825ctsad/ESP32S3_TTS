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
#include "ws_cfg.h"
#include "ws_cfg_internal.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

#define TAG "WS_CFG"

#define WS_URI_MAX          256
#define WS_BUFFER_SIZE      2048   /* > 1024B PCM 帧 + JSON 帧，避免分片 */
#define WS_NET_TIMEOUT_MS   10000
#define WS_TASK_STACK       8192
#define WS_SEND_TIMEOUT_MS  100

static esp_websocket_client_handle_t s_client = NULL;
static char s_uri[WS_URI_MAX] = {0};

static EventGroupHandle_t s_evt = NULL;
static ringbuf_handle_t s_ring = NULL;

/* ================================================================ */
/*  事件回调                                                         */
/* ================================================================ */

static void ws_event_cb(void *arg, esp_event_base_t base,
                        int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected: %s", s_uri);
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
                if (strcmp(type->valuestring, "done") == 0) {
                    ESP_LOGI(TAG, "Received done");
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_TTS_DONE);
                } else if (strcmp(type->valuestring, "error") == 0) {
                    ESP_LOGW(TAG, "Received error: %.*s", data->data_len, data->data_ptr);
                    if (s_evt) xEventGroupSetBits(s_evt, WS_EVT_TTS_ERROR);
                } else if (strcmp(type->valuestring, "tts_start") == 0) {
                    ESP_LOGI(TAG, "Received tts_start");
                } else {
                    ESP_LOGI(TAG, "Unknown control: %.*s", data->data_len, data->data_ptr);
                }
            }
            cJSON_Delete(root);
        } else if (data->op_code == WS_TRANSPORT_OPCODES_BINARY ||
                   data->op_code == WS_TRANSPORT_OPCODES_CONT) {
            /* 二进制帧 = PCM16 音频 → 播放缓冲（非阻塞，满则丢弃） */
            int n = rb_write(s_ring, (char *)data->data_ptr, data->data_len, 0);
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
    /* 归一化 https:// → wss://，http:// → ws://（mqtt_cfg 拼出 https://） */
    if (strncmp(uri, "https://", 8) == 0) {
        snprintf(s_uri, sizeof(s_uri), "wss://%s", uri + 8);
    } else if (strncmp(uri, "http://", 7) == 0) {
        snprintf(s_uri, sizeof(s_uri), "ws://%s", uri + 7);
    } else {
        snprintf(s_uri, sizeof(s_uri), "%s", uri);
    }
    ESP_LOGI(TAG, "WS uri saved: %s", s_uri);
    return ESP_OK;
}

bool ws_cfg_has_uri(void)
{
    return s_uri[0] != '\0';
}

/* ================================================================ */
/*  transport API（voice_session 使用）                              */
/* ================================================================ */

void ws_cfg_attach(EventGroupHandle_t evt, ringbuf_handle_t ring)
{
    s_evt = evt;
    s_ring = ring;
}

esp_err_t ws_cfg_connect(void)
{
    if (s_client != NULL) {
        ESP_LOGW(TAG, "Already connecting/connected");
        return ESP_ERR_INVALID_STATE;
    }
    if (!ws_cfg_has_uri()) {
        ESP_LOGE(TAG, "No uri saved");
        return ESP_ERR_INVALID_STATE;
    }

    const esp_websocket_client_config_t cfg = {
        .uri = s_uri,
        .buffer_size = WS_BUFFER_SIZE,
        .network_timeout_ms = WS_NET_TIMEOUT_MS,
        .task_stack = WS_TASK_STACK,
        .disable_auto_reconnect = true,   /* 连接生命周期 = 会话，手动控制 */
        .reconnect_timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,  /* 公网 CA 根证书 bundle */
    };

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
    ESP_LOGI(TAG, "WS connecting: %s", s_uri);
    return ESP_OK;
}

esp_err_t ws_cfg_disconnect(void)
{
    if (s_client == NULL) return ESP_OK;
    esp_websocket_client_close(s_client, 1000);
    esp_websocket_client_stop(s_client);
    esp_websocket_client_destroy(s_client);
    s_client = NULL;
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
    int r = esp_websocket_client_send_text(s_client, json,
                                           (int)strlen(json),
                                           pdMS_TO_TICKS(WS_SEND_TIMEOUT_MS));
    return r < 0 ? ESP_FAIL : ESP_OK;
}

esp_err_t ws_cfg_send_pcm(const uint8_t *data, int len)
{
    if (s_client == NULL || !esp_websocket_client_is_connected(s_client)) {
        ESP_LOGW(TAG, "Not connected, pcm dropped");
        return ESP_ERR_INVALID_STATE;
    }
    int r = esp_websocket_client_send_bin(s_client, (const char *)data, len,
                                          pdMS_TO_TICKS(WS_SEND_TIMEOUT_MS));
    return r < 0 ? ESP_FAIL : ESP_OK;
}
