/* mqtt.c  精简版：纯任务，无软件定时器 */
#include <stdio.h>
#include "mqtt_cfg.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "relay.h"
#include "TTS.h"
#include "cJSON.h"

#define TAG                    "MQTT_CLIENT"
#define MQTT_BROKER_URI        "mqtt://192.168.1.200"
#define MQTT_BROKER_PORT       1883

#define MQTT_USERNAME          "esp32s3"
#define MQTT_PASSWORD          ""
#define MQTT_MAX_PAYLOAD       2048    /* 限制单条消息最大长度，防止 OOM */

/* 基于芯片 MAC 地址的唯一标识：完整 12 位十六进制 MAC */
static char mqtt_client_id[13] = "000000000000";
/* 设备专属主题：cmd/<mac> 和 data/<mac>，避免多设备互相干扰 */
static char mqtt_topic_cmd[24];       /* "cmd/b81f3fb86280" */
static char mqtt_topic_data[24];      /* "data/b81f3fb86280" */
static char mqtt_topic_audio[24];     /* "audio/b81f3fb86280" */

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;  /* 跨任务访问，加 volatile */
static int reconnect_attempts = 0;

/* ---------- MQTT 事件回调 ---------- */
static void mqtt_event_callback(void *handler_args,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t data = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        /* 订阅设备专属指令主题：cmd/<mac> */
        esp_mqtt_client_subscribe_single(mqtt_client, mqtt_topic_cmd, 1);
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED, subscribed: %s", mqtt_topic_cmd);
        mqtt_connected = true;
        reconnect_attempts = 0;
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
            
            // 根据返回码处理不同的拒绝原因
            switch (data->error_handle->connect_return_code) {
                case 1:
                    ESP_LOGE(TAG, "Unacceptable protocol version");
                    break;
                case 2:
                    ESP_LOGE(TAG, "Identifier rejected");
                    break;
                case 3:
                    ESP_LOGE(TAG, "Server unavailable");
                    break;
                case 4:
                    ESP_LOGE(TAG, "Bad username or password");
                    break;
                case 5:
                    ESP_LOGE(TAG, "Not authorized");
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown refusal reason");
                    break;
            }
        }
        break;

        case MQTT_EVENT_DATA: 
            ESP_LOGI(TAG, "Received message on topic: %.*s", data->topic_len, data->topic);
            ESP_LOGI(TAG, "Message content: %.*s", data->data_len, data->data);
        
            /* 限制 payload 长度，防止恶意大包耗尽堆内存 */
            if (data->data_len > MQTT_MAX_PAYLOAD) {
                ESP_LOGW(TAG, "Payload too large (%d > %d), discarded", data->data_len, MQTT_MAX_PAYLOAD);
                break;
            }

            // 1. 转换消息为字符串（添加终止符）
            char *msg_str = (char *)malloc(data->data_len + 1);
            if (msg_str == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for message");
                break;
            }
            memcpy(msg_str, data->data, data->data_len);
            msg_str[data->data_len] = '\0';
        
            // 2. 解析JSON
            cJSON *root = cJSON_Parse(msg_str);
            if (root == NULL) {
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL) {
                    ESP_LOGE(TAG, "JSON parse error before: %s", error_ptr);
                }
                free(msg_str);
                break;
            }
        
            // // 3. 解析并处理每个字段
            // // 3.1 继电器控制（relay: "on"/"off"）
            // cJSON *relay = cJSON_GetObjectItemCaseSensitive(root, "relay");
            // if (cJSON_IsString(relay) && relay->valuestring != NULL) {
            //     if (strcmp(relay->valuestring, "on") == 0) {
            //         relay_on();
            //         ESP_LOGI(TAG, "Relay turned ON");
            //     } else if (strcmp(relay->valuestring, "off") == 0) {
            //         relay_off();
            //         ESP_LOGI(TAG, "Relay turned OFF");
            //     } else {
            //         ESP_LOGE(TAG, "Invalid relay value: %s (expected 'on' or 'off')", relay->valuestring);
            //     }
            // } else {
            //     ESP_LOGD(TAG, "No relay field in message");
            // }
            // 3.2 TTS 语音播报（tts: "文本内容"）
            cJSON *tts = cJSON_GetObjectItemCaseSensitive(root, "tts");
            if (cJSON_IsString(tts) && tts->valuestring != NULL) {
                tts_speak_async(tts->valuestring);
                ESP_LOGI(TAG, "TTS queued: %s", tts->valuestring);
            }

            // 4. 释放资源
            cJSON_Delete(root);
            free(msg_str);
            break;  

    default:
        break;
    }
}

/* ---------- 启动 MQTT ---------- */
static void mqtt_start(void)
{
    /* 已有客户端则不重复创建，依赖 ESP-MQTT 内部自动重连 */
    if (mqtt_client != NULL) {
        return;
    }

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
            .disable_auto_reconnect = false,   /* 启用自动重连 */
        },
        .task = {
            .priority = 5,
            .stack_size = 8192,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client init failed (NULL)");
        return;
    }
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_callback, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
}


/* ---------- 发布传感器数据 ---------- */
static void publish_sensor_data(void)
{
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "Not connected, skipping publish");
        return;
    }

    /*  构造 JSON（这里用整数/浮点均可） */
    static char json[256];
    int len = snprintf(json, sizeof(json),
        "{\"timestamp\":%lu,\"device\":\"%s\"}",
        
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),mqtt_client_id);

    if (len > 0 && len < sizeof(json)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_data, json, 0, 0, 0);
        if (msg_id < 0) 
        {
            ESP_LOGE(TAG, "Publish failed: %d", msg_id);
        } 
        else 
        {
            ESP_LOGI(TAG, "Published: %s", json);
        }
    }
}

/* ---------- 主任务 ---------- */
void mqtt_task(void *pvParameters)
{
    /* 基于芯片 MAC 生成唯一 client_id 和设备专属主题 */
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(mqtt_client_id, sizeof(mqtt_client_id),
                 "%02x%02x%02x%02x%02x%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    /* cmd/<mac>  —— 下行指令主题（设备订阅） */
    /* data/<mac> —— 上行数据主题（设备发布） */
    /* audio/<mac> —— 上行音频主题（设备发布 Base64 PCM） */
    snprintf(mqtt_topic_cmd,   sizeof(mqtt_topic_cmd),   "cmd/%s",   mqtt_client_id);
    snprintf(mqtt_topic_data,  sizeof(mqtt_topic_data),  "data/%s",  mqtt_client_id);
    snprintf(mqtt_topic_audio, sizeof(mqtt_topic_audio), "audio/%s", mqtt_client_id);
    ESP_LOGI(TAG, "Client ID: %s | cmd: %s | data: %s | audio: %s",
             mqtt_client_id, mqtt_topic_cmd, mqtt_topic_data, mqtt_topic_audio);

    // 等待网络连接
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    mqtt_start();

    while (1) {
        /* 自动重连由 ESP-MQTT 内部处理，这里只负责发布数据 */
        if (mqtt_connected) {
            publish_sensor_data();
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ---------- 发布音频数据（Base64 PCM） ---------- */
void mqtt_publish_audio(const char *b64_data, size_t b64_len)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "Not connected, audio discarded");
        return;
    }
    /* QoS 0，不保留，音频是实时流数据 */
    int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_audio,
                                          b64_data, (int)b64_len, 0, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Audio publish failed: %d", msg_id);
    } else {
        ESP_LOGI(TAG, "Audio published: %d bytes (b64)", (int)b64_len);
    }
}
