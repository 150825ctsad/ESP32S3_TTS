/* mqtt.c  精简版：纯任务，无软件定时器 */
#include <stdio.h>
#include "mqtt_cfg.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "relay.h"
#include "mbedtls/base64.h"
#include "cJSON.h"

#define TAG                    "MQTT_CLIENT"
#define MQTT_BROKER_URI        "tcp://192.168.2.112"
#define MQTT_BROKER_PORT       1883

#define MQTT_CLIENT_ID         "esp32s3"
#define MQTT_USERNAME          "esp32s3"
#define MQTT_PASSWORD          ""
#define MQTT_TOPIC_SENSOR      "test/ESP-IDF/SENSOR_DATA"
#define MQTT_TOPIC_COMMAND     "test/ESP-IDF/COMMAND"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
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
        esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_SENSOR, 0);
        esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_COMMAND, 1);
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        reconnect_attempts = 0; // 重置重连尝试计数
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_ERROR:
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
        
            // 3. 解析并处理每个字段
            // 3.1 继电器控制（relay: "on"/"off"）
            cJSON *relay = cJSON_GetObjectItemCaseSensitive(root, "relay");
            if (cJSON_IsString(relay) && relay->valuestring != NULL) {
                if (strcmp(relay->valuestring, "on") == 0) {
                    relay_on();
                    ESP_LOGI(TAG, "Relay turned ON");
                } else if (strcmp(relay->valuestring, "off") == 0) {
                    relay_off();
                    ESP_LOGI(TAG, "Relay turned OFF");
                } else {
                    ESP_LOGE(TAG, "Invalid relay value: %s (expected 'on' or 'off')", relay->valuestring);
                }
            } else {
                ESP_LOGW(TAG, "Missing or invalid 'relay' field");
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
    // 如果已有客户端实例，先销毁它
    if (mqtt_client != NULL) {
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 等待资源释放
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .address.port = MQTT_BROKER_PORT,
        },
        .credentials = {
            .client_id = MQTT_CLIENT_ID,
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        },
        .session = {
            .keepalive = 60, // 减少心跳间隔
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 10000,
            .reconnect_timeout_ms = 5000,
            .disable_auto_reconnect = false, // 启用自动重连
        },
        .task = {
            .priority = 5,
            .stack_size = 8192, // 增加栈大小
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_callback, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
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
        "{\"timestamp\":%lu,\"device\":\"%s\",\"fw\":\"1.0.0\"}",
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),MQTT_CLIENT_ID);

    if (len > 0 && len < sizeof(json)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SENSOR, json, 0, 0, 0);
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
    // 等待网络连接
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    mqtt_start();

    while (1) {
        /* 未连接就重连 */
        if (!mqtt_connected) {
            reconnect_attempts++;
            ESP_LOGW(TAG, "MQTT reconnecting... (attempt %d)", reconnect_attempts);
            
            // 指数退避策略
            int delay_ms = 1000 * (1 << (reconnect_attempts > 6 ? 6 : reconnect_attempts));
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            
            mqtt_start();
            continue;
        }

        /* 每 1 秒发布一次数据 */
        publish_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
