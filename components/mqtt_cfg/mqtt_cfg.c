#include <stdio.h>
#include <string.h>
#include "mqtt_cfg.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "relay.h"
#include "TTS.h"
#include "cJSON.h"

#define TAG                    "MQTT_CLIENT"
#define MQTT_BROKER_URI        "mqtt://192.168.1.200"
#define MQTT_BROKER_PORT       1883

#define MQTT_CLIENT_ID         "esp32s3"
#define MQTT_USERNAME          "esp32s3"
#define MQTT_PASSWORD          ""
#define MQTT_TOPIC_SENSOR      "test/ESP-IDF/SENSOR_DATA"
#define MQTT_TOPIC_COMMAND     "test/ESP-IDF/COMMAND"

// 消息队列：MQTT回调转发指令，避免在回调中执行业务
#define MQTT_CMD_QUEUE_LEN     8
typedef struct {
    char topic[64];
    char payload[256];
} mqtt_cmd_msg_t;
static QueueHandle_t mqtt_cmd_queue = NULL;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static int reconnect_attempts = 0;
static bool has_subscribed = false;

/* ---------- 从队列处理收到的MQTT指令（独立任务上下文，安全） ---------- */
static void handle_mqtt_command(const mqtt_cmd_msg_t *msg)
{
    ESP_LOGI(TAG, "Topic: %s, Payload: %s", msg->topic, msg->payload);

    cJSON *root = cJSON_Parse(msg->payload);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(TAG, "JSON parse error before: %s", error_ptr);
        }
        return;
    }

    // 继电器1
    cJSON *relay1 = cJSON_GetObjectItemCaseSensitive(root, "relay1");
    if (cJSON_IsString(relay1) && relay1->valuestring != NULL) {
        if (strcmp(relay1->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY1);
            ESP_LOGI(TAG, "Relay1 ON");
        } else if (strcmp(relay1->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY1);
            ESP_LOGI(TAG, "Relay1 OFF");
        }
    }
    // 继电器2
    cJSON *relay2 = cJSON_GetObjectItemCaseSensitive(root, "relay2");
    if (cJSON_IsString(relay2) && relay2->valuestring != NULL) {
        if (strcmp(relay2->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY2);
            ESP_LOGI(TAG, "Relay2 ON");
        } else if (strcmp(relay2->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY2);
            ESP_LOGI(TAG, "Relay2 OFF");
        }
    }
    // 继电器3
    cJSON *relay3 = cJSON_GetObjectItemCaseSensitive(root, "relay3");
    if (cJSON_IsString(relay3) && relay3->valuestring != NULL) {
        if (strcmp(relay3->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY3);
            ESP_LOGI(TAG, "Relay3 ON");
        } else if (strcmp(relay3->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY3);
            ESP_LOGI(TAG, "Relay3 OFF");
        }
    }
    // 继电器4
    cJSON *relay4 = cJSON_GetObjectItemCaseSensitive(root, "relay4");
    if (cJSON_IsString(relay4) && relay4->valuestring != NULL) {
        if (strcmp(relay4->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY4);
            ESP_LOGI(TAG, "Relay4 ON");
        } else if (strcmp(relay4->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY4);
            ESP_LOGI(TAG, "Relay4 OFF");
        }
    }
    // 继电器5
    cJSON *relay5 = cJSON_GetObjectItemCaseSensitive(root, "relay5");
    if (cJSON_IsString(relay5) && relay5->valuestring != NULL) {
        if (strcmp(relay5->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY5);
            ESP_LOGI(TAG, "Relay5 ON");
        } else if (strcmp(relay5->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY5);
            ESP_LOGI(TAG, "Relay5 OFF");
        }
    }
    // 继电器6
    cJSON *relay6 = cJSON_GetObjectItemCaseSensitive(root, "relay6");
    if (cJSON_IsString(relay6) && relay6->valuestring != NULL) {
        if (strcmp(relay6->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY6);
            ESP_LOGI(TAG, "Relay6 ON");
        } else if (strcmp(relay6->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY6);
            ESP_LOGI(TAG, "Relay6 OFF");
        }
    }
    // 继电器7
    cJSON *relay7 = cJSON_GetObjectItemCaseSensitive(root, "relay7");
    if (cJSON_IsString(relay7) && relay7->valuestring != NULL) {
        if (strcmp(relay7->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY7);
            ESP_LOGI(TAG, "Relay7 ON");
        } else if (strcmp(relay7->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY7);
            ESP_LOGI(TAG, "Relay7 OFF");
        }
    }
    // 继电器8
    cJSON *relay8 = cJSON_GetObjectItemCaseSensitive(root, "relay8");
    if (cJSON_IsString(relay8) && relay8->valuestring != NULL) {
        if (strcmp(relay8->valuestring, "on") == 0) {
            relay_on(GPIO_NUM_RELAY8);
            ESP_LOGI(TAG, "Relay8 ON");
        } else if (strcmp(relay8->valuestring, "off") == 0) {
            relay_off(GPIO_NUM_RELAY8);
            ESP_LOGI(TAG, "Relay8 OFF");
        }
    }

    // TTS语音播报
    cJSON *tts = cJSON_GetObjectItemCaseSensitive(root, "tts");
    if (cJSON_IsString(tts) && tts->valuestring != NULL) {
        tts_speak_async(tts->valuestring);
        ESP_LOGI(TAG, "TTS queued: %s", tts->valuestring);
    }

    cJSON_Delete(root);
}

/* ---------- MQTT 事件回调 ---------- */
static void mqtt_event_callback(void *handler_args,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t data = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        has_subscribed = false;
        mqtt_connected = true;
        reconnect_attempts = 0;
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        if (data->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Transport error: errno=%d", data->error_handle->esp_transport_sock_errno);
        } else if (data->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused code: %d", data->error_handle->connect_return_code);
        }
        break;

    case MQTT_EVENT_DATA: {
        mqtt_cmd_msg_t cmd;
        memset(&cmd, 0, sizeof(cmd));

        // 截断防止溢出
        size_t topic_cpy_len = (data->topic_len >= sizeof(cmd.topic)) ? (sizeof(cmd.topic)-1) : data->topic_len;
        memcpy(cmd.topic, data->topic, topic_cpy_len);

        size_t data_cpy_len = (data->data_len >= sizeof(cmd.payload)) ? (sizeof(cmd.payload)-1) : data->data_len;
        memcpy(cmd.payload, data->data, data_cpy_len);

        // 非阻塞入队，队列满直接丢弃指令，防止阻塞回调
        xQueueSend(mqtt_cmd_queue, &cmd, 0);
        break;
    }
    default:
        break;
    }
}

/* ---------- 启动 MQTT ---------- */
esp_err_t mqtt_start(void)
{
    if (mqtt_client != NULL) {
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        vTaskDelay(pdMS_TO_TICKS(200));
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
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_callback, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
        mqtt_client = NULL;
    }
    return err;
}

/* ---------- 发布传感器数据 ---------- */
static void publish_sensor_data(void)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TAG, "Not connected, skip publish");
        return;
    }

    static char json[256];
    int len = snprintf(json, sizeof(json),
        "{\"timestamp\":%lu,\"device\":\"%s\","
        "\"relay1\":\"%s\",\"relay2\":\"%s\",\"relay3\":\"%s\",\"relay4\":\"%s\","
        "\"relay5\":\"%s\",\"relay6\":\"%s\",\"relay7\":\"%s\",\"relay8\":\"%s\","
        "\"fw\":\"1.0.0\"}",
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        MQTT_CLIENT_ID,
        relay_state1 ? "on" : "off",
        relay_state2 ? "on" : "off",
        relay_state3 ? "on" : "off",
        relay_state4 ? "on" : "off",
        relay_state5 ? "on" : "off",
        relay_state6 ? "on" : "off",
        relay_state7 ? "on" : "off",
        relay_state8 ? "on" : "off");

    if (len > 0 && len < sizeof(json)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SENSOR, json, 0, 0, 0);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Publish failed, msg_id=%d", msg_id);
        } else {
            ESP_LOGD(TAG, "Published sensor data");
        }
    }
}

/* ---------- MQTT主任务 ---------- */
void mqtt_task(void *pvParameters)
{
    // 创建指令队列
    mqtt_cmd_queue = xQueueCreate(MQTT_CMD_QUEUE_LEN, sizeof(mqtt_cmd_msg_t));
    if (mqtt_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Create mqtt cmd queue failed!");
        vTaskDelete(NULL);
    }

    // 等待网络就绪
    vTaskDelay(pdMS_TO_TICKS(5000));
    mqtt_start();

    mqtt_cmd_msg_t cmd_buf;

    while (1) {
        // 连接成功后执行订阅
        if (mqtt_connected && !has_subscribed && mqtt_client != NULL) {
            esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_SENSOR, 0);
            esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_COMMAND, 1);
            has_subscribed = true;
            ESP_LOGI(TAG, "Subscribe topics success");
        }

        // 断线重连逻辑
        if (!mqtt_connected) {
            reconnect_attempts++;
            ESP_LOGW(TAG, "MQTT reconnect attempt: %d", reconnect_attempts);
            int delay_ms = 1000 * (reconnect_attempts > 6 ? 6 : reconnect_attempts);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            mqtt_start();
            continue;
        }

        // 处理收到的下发指令
        if (xQueueReceive(mqtt_cmd_queue, &cmd_buf, pdMS_TO_TICKS(100)) == pdPASS) {
            handle_mqtt_command(&cmd_buf);
        }

        // 定时上报传感器/继电器状态
        publish_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}