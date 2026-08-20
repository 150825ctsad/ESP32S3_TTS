/* mqtt.c  -- MQTT 业务逻辑（环境数据采集）
 *
 * 主题设计：
 *   cmd/<mac>    下行指令（设备订阅 QoS1）：{"vol":50} / {"ws":"/ws/..."}
 *   status/<mac>   上行环境数据（设备发布 QoS0）：WiFi 信息 RSSI/IP/SSID/uptime/vol
 *   online       上线通知（通用主题，QoS2，retained，消息体含 device 字段）
 *
 * 业务流程：
 *   1. 连接后立即上报设备ID（QoS2, retained）
 *   2. 周期上报 WiFi 信息（环境数据）
 *   3. 下发 {"vol":50} → 设置音量 → 返回 {"vol":"ok","value":50}
 *   4. 下发含 /ws/ 路径字段 → 保存地址并立刻连 WS 播 TTS，播完返回 {"msgId":"..."}
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mqtt_cfg.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_board_init.h"
#include "ws_cfg.h"
#include "cJSON.h"

#define TAG                    "MQTT_CLIENT"
#define MQTT_BROKER_URI        "mqtt://mqtt-xiaoyi.gejia.tech" //mqtt:mqtt-xiaoyi.gejia.tech wss:https://iot-xiaoyi.gejia.tech
#define MQTT_BROKER_PORT       1883

#define MQTT_USERNAME          "esp32s3"
#define MQTT_PASSWORD          ""
#define MQTT_MAX_PAYLOAD       2048

/* 基于芯片 MAC 地址的唯一标识 */
static char mqtt_client_id[13] = "000000000000";

/* 设备专属主题 */
static char mqtt_topic_cmd[24];        /* "cmd/b81f3fb86280"   下行指令 */
static char mqtt_topic_data[24];      /* "status/b81f3fb86280"  上行环境数据 */
/* online 为通用主题，所有设备共用，消息体中携带 device 字段区分 */
#define MQTT_TOPIC_ONLINE  "online"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

/* ================================================================ */
/*  上行：发布函数                                                   */
/* ================================================================ */

/* 初始上报设备ID（QoS2, retained）—— 设备上线时调用一次 */
static void publish_device_online(void)
{
    if (mqtt_client == NULL) return;

    esp_netif_ip_info_t ip_info = {0};
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    char json[128];
    int len = snprintf(json, sizeof(json),
        "{\"device\":\"%s\",\"ip\":\"" IPSTR "\"}",
        mqtt_client_id, IP2STR(&ip_info.ip));
    if (len > 0 && len < sizeof(json)) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ONLINE, json, len, 2, 1);
        ESP_LOGI(TAG, "Online: %s", json);
    }
}

/* 周期上报 WiFi 信息（QoS0） */
static void publish_wifi_info(void)
{
    if (!mqtt_connected || mqtt_client == NULL) return;

    wifi_ap_record_t ap;
    memset(&ap, 0, sizeof(ap));
    bool wifi_ok = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

    esp_netif_ip_info_t ip_info = {0};
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    int8_t rssi = wifi_ok ? ap.rssi : 0;
    char ssid[33] = {0};
    if (wifi_ok) {
        strncpy(ssid, (char *)ap.ssid, 32);
    }

    int vol = 0;
    esp_audio_get_play_vol(&vol);

    char json[256];
    int len = snprintf(json, sizeof(json),
        "{\"device\":\"%s\",\"rssi\":%d,\"ip\":\"" IPSTR "\",\"ssid\":\"%s\",\"uptime\":%lu,\"vol\":%d}",
        mqtt_client_id,
        (int)rssi,
        IP2STR(&ip_info.ip),
        ssid,
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000),
        vol);

    if (len > 0 && len < sizeof(json)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_data, json, len, 0, 0);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Publish failed: %d", msg_id);
        } else {
            ESP_LOGI(TAG, "Published: %s", json);
        }
    }
}

/* 云端 TTS 播完 → 把下发的 msgId 回传 */
static void on_push_tts_done(const char *msg_id, bool ok)
{
    if (!mqtt_connected || mqtt_client == NULL || msg_id == NULL || !msg_id[0]) {
        return;
    }
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return;
    cJSON_AddStringToObject(root, "msgId", msg_id);
    cJSON_AddStringToObject(root, "tts", ok ? "ok" : "fail");
    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (js == NULL) return;
    int len = (int)strlen(js);
    esp_mqtt_client_publish(mqtt_client, mqtt_topic_data, js, len, 1, 0);
    ESP_LOGI(TAG, "Push done ack: %s", js);
    cJSON_free(js);
}

/* ================================================================ */
/*  下行：指令处理                                                   */
/* ================================================================ */

static void handle_command(const char *msg_str)
{
    cJSON *root = cJSON_Parse(msg_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON parse error");
        return;
    }

    /* 1. 音量控制  */
    cJSON *vol = cJSON_GetObjectItemCaseSensitive(root, "vol");
    int v = -1;
    if (cJSON_IsNumber(vol)) {
        v = vol->valueint;
    } else if (cJSON_IsString(vol) && vol->valuestring != NULL) {
        v = atoi(vol->valuestring);
    }
    if (v >= 0) {
        if (v > 100) v = 100;
        esp_audio_set_play_vol(v);
        ESP_LOGI(TAG, "Volume set: %d", v);
        /* 返回确认 */
        char json[64];
        int len = snprintf(json, sizeof(json), "{\"vol\":\"ok\",\"value\":%d}", v);
        if (len > 0 && len < sizeof(json)) {
            esp_mqtt_client_publish(mqtt_client, mqtt_topic_data, json, len, 1, 0);
        }
    }

    /* 含 /ws/ 路径：保存地址并立刻连 WS 播放，完成后回传 msgId */
    const char *msgid = NULL;
    const char *tts_text = NULL;
    cJSON *jmsgid = cJSON_GetObjectItemCaseSensitive(root, "msgId");
    if (cJSON_IsString(jmsgid) && jmsgid->valuestring) msgid = jmsgid->valuestring;
    cJSON *jtts = cJSON_GetObjectItemCaseSensitive(root, "tts");
    if (cJSON_IsString(jtts) && jtts->valuestring) tts_text = jtts->valuestring;

    bool got_ws = false;
    cJSON *iter = NULL;
    cJSON_ArrayForEach(iter, root) {
        if (cJSON_IsString(iter) && iter->valuestring != NULL) {
            if (strncmp(iter->valuestring, "/ws/", 4) == 0) {
                char full_ws[256];
                int n = snprintf(full_ws, sizeof(full_ws), "https://iot.gejia.tech%s", iter->valuestring);
                if (n > 0 && n < (int)sizeof(full_ws)) {
                    ESP_LOGI(TAG, "Detected ws path in cmd (field=%s): %s", iter->string ? iter->string : "<anon>", full_ws);
                    if (ws_cfg_set_uri(full_ws) == ESP_OK) {
                        got_ws = true;
                    }
                }
            }
        }
    }
    if (got_ws || (tts_text && tts_text[0])) {
        ws_cfg_request_push(msgid, tts_text);
    }

    cJSON_Delete(root);
}

/* ================================================================ */
/*  MQTT 事件回调                                                    */
/* ================================================================ */

static void mqtt_event_callback(void *handler_args,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t data = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_subscribe_single(mqtt_client, mqtt_topic_cmd, 1);
        mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED, subscribed: %s", mqtt_topic_cmd);
        /* 连接后立即上报设备ID（QoS2, retained） */
        publish_device_online();
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        if (data->error_handle == NULL) {
            ESP_LOGE(TAG, "MQTT error (no error_handle)");
            break;
        }
        if (data->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Transport error: errno=%d", data->error_handle->esp_transport_sock_errno);
        } else if (data->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused: %d", data->error_handle->connect_return_code);
        }
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Received: %.*s", data->data_len, data->data);

        if (data->data_len > MQTT_MAX_PAYLOAD) {
            ESP_LOGW(TAG, "Payload too large (%d), discarded", data->data_len);
            break;
        }

        char *msg_str = (char *)malloc(data->data_len + 1);
        if (msg_str == NULL) {
            ESP_LOGE(TAG, "malloc failed");
            break;
        }
        memcpy(msg_str, data->data, data->data_len);
        msg_str[data->data_len] = '\0';

        handle_command(msg_str);

        free(msg_str);
        break;

    default:
        break;
    }
}

/* ================================================================ */
/*  启动 MQTT                                                        */
/* ================================================================ */

static void mqtt_start(void)
{
    if (mqtt_client != NULL) return;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .address.port = MQTT_BROKER_PORT,
        },
        .credentials = {
            .client_id = mqtt_client_id,
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        },
        .session = {
            .keepalive = 60,
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 10000,
            .reconnect_timeout_ms = 5000,
            .disable_auto_reconnect = false,
        },
        .task = {
            .priority = 5,
            .stack_size = 8192,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client init failed");
        return;
    }
    ws_cfg_set_push_done_cb(on_push_tts_done);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_callback, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
}

/* ================================================================ */
/*  主任务                                                           */
/* ================================================================ */

void mqtt_task(void *pvParameters)
{
    /* 生成唯一 client_id 和设备专属主题 */
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(mqtt_client_id, sizeof(mqtt_client_id),
                 "%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    snprintf(mqtt_topic_cmd,    sizeof(mqtt_topic_cmd),    "cmd/%s",    mqtt_client_id);
    snprintf(mqtt_topic_data,   sizeof(mqtt_topic_data),   "status/%s",   mqtt_client_id);
    ESP_LOGI(TAG, "Client ID: %s | cmd: %s | status: %s | online: %s",
             mqtt_client_id, mqtt_topic_cmd, mqtt_topic_data, MQTT_TOPIC_ONLINE);

    /* 等待网络连接 */
    vTaskDelay(pdMS_TO_TICKS(20000));

    mqtt_start();

    while (1) {
        if (mqtt_connected) {
            publish_wifi_info();
        }
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

