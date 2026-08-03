#include <stdio.h>
#include <string.h>
#include "mqtt_cfg.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "relay.h"
#include "driver/gpio.h"
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

#define MQTT_CMD_QUEUE_LEN     16
typedef struct {
    char topic[64];
    char payload[256];
} mqtt_cmd_msg_t;
static QueueHandle_t mqtt_cmd_queue = NULL;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static int reconnect_attempts = 0;
static bool has_subscribed = false;

/* ================================================================ */
/*  handle_mqtt_command  -- parse JSON, control relay + TTS          */
/* ================================================================ */
static void handle_mqtt_command(const mqtt_cmd_msg_t *msg)
{
    ESP_LOGI(TAG, "Topic: %s, Payload: %s", msg->topic, msg->payload);

    cJSON *root = cJSON_Parse(msg->payload);
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON parse error: %s", cJSON_GetErrorPtr());
        return;
    }

    /* ---- relay control: {"box1":"on"}, {"box2":"off"}, ... ---- */
    for (int i = 1; i <= RELAY_COUNT; i++)
    {
        char key[8];
        snprintf(key, sizeof(key), "box%d", i);
            cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
                if (!cJSON_IsString(item) || item->valuestring == NULL)
                    continue;

                if(strcmp(item->valuestring,"on") != 0)
                    continue;

            uint8_t state = box_state[i-1];
                if(state == 1){
                    ESP_LOGW(TAG,"Box%d already ON, reject",i);
                    continue;
                }
        ESP_LOGI(TAG,"Box%d trigger",i);
        box_trigger(i,1);
        tts_speak_async("门已开请尽快取货");
        vTaskDelay(pdMS_TO_TICKS(500));
        tts_speak_async("取货后请关门");
}

/* ---- TTS speech ---- */
    cJSON *tts = cJSON_GetObjectItemCaseSensitive(root, "tts");
    if (cJSON_IsString(tts) && tts->valuestring) {
        tts_speak_async(tts->valuestring);
        ESP_LOGI(TAG, "TTS queued: %s", tts->valuestring);
    }

    cJSON_Delete(root);
}

/* ================================================================ */
/*  MQTT event callback  -- enqueue only, no blocking                */
/* ================================================================ */
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
            ESP_LOGE(TAG, "Transport error: errno=%d",
                     data->error_handle->esp_transport_sock_errno);
        } else if (data->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused code: %d",
                     data->error_handle->connect_return_code);
        }
        break;

    case MQTT_EVENT_DATA: {
        mqtt_cmd_msg_t cmd;
        memset(&cmd, 0, sizeof(cmd));

        size_t tc = (data->topic_len >= sizeof(cmd.topic))
                        ? (sizeof(cmd.topic) - 1) : data->topic_len;
        memcpy(cmd.topic, data->topic, tc);

        size_t dc = (data->data_len >= sizeof(cmd.payload))
                        ? (sizeof(cmd.payload) - 1) : data->data_len;
        memcpy(cmd.payload, data->data, dc);

        xQueueSend(mqtt_cmd_queue, &cmd, pdMS_TO_TICKS(10));
        break;
    }
    default:
        break;
    }
}

/* ================================================================ */
/*  mqtt_start  -- create and connect MQTT client                    */
/* ================================================================ */
esp_err_t mqtt_start(void)
{
    if (mqtt_client) {
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    const esp_mqtt_client_config_t cfg = {
        .broker = {
            .address.uri  = MQTT_BROKER_URI,
            .address.port = MQTT_BROKER_PORT,
        },
        .credentials = {
            .client_id = MQTT_CLIENT_ID,
            .username   = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        },
        .session = {
            .keepalive              = 60,
            .disable_clean_session  = false,
        },
        .network = {
            .timeout_ms             = 10000,
            .reconnect_timeout_ms   = 5000,
            .disable_auto_reconnect = false,
        },
        .task = {
            .priority   = 5,
            .stack_size = 8192,
        },
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_callback, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
        mqtt_client = NULL;
    }
    return err;
}

/* ================================================================ */
/*  publish_sensor_data  -- periodic status report                    */
/* ================================================================ */
static void publish_sensor_data(void)
{
    if (!mqtt_connected || !mqtt_client) return;

    /* Build box states with loop */
    char boxes[256];
    int  pos = 0;
    for (int i = 1; i <= RELAY_COUNT; i++) {
        pos += snprintf(boxes + pos, sizeof(boxes) - pos,
                        "\"box%d\":\"%s\"%s",
                        i, box_state[i - 1] ? "on" : "off",
                        (i < RELAY_COUNT) ? "," : "");
    }

    char json[512];
    int len = snprintf(json, sizeof(json),
        "{\"timestamp\":%lu,\"device\":\"%s\",%s,\"fw\":\"1.0.0\"}",
        (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        MQTT_CLIENT_ID, boxes);

    if (len > 0 && len < sizeof(json)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SENSOR,
                                              json, 0, 0, 0);
        if (msg_id < 0)
            ESP_LOGE(TAG, "Publish failed, msg_id=%d", msg_id);
        else
            ESP_LOGD(TAG, "Published sensor data");
    }
}

/* ================================================================ */
/*  Tasks                                                             */
/* ================================================================ */
static void mqtt_listen_task(void *arg)
{
    mqtt_cmd_msg_t cmd;
    while (1) {
        if (xQueueReceive(mqtt_cmd_queue, &cmd, portMAX_DELAY) == pdPASS)
            handle_mqtt_command(&cmd);
    }
}

static void mqtt_publish_task(void *arg)
{
    while (1) {
        if (mqtt_connected && !has_subscribed && mqtt_client) {
            esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_SENSOR, 0);
            esp_mqtt_client_subscribe_single(mqtt_client, MQTT_TOPIC_COMMAND, 1);
            has_subscribed = true;
            ESP_LOGI(TAG, "Subscribed to topics");
        }
        if (!mqtt_connected) {
            reconnect_attempts++;
            int delay_ms = 1000 * (reconnect_attempts > 6 ? 6 : reconnect_attempts);
            ESP_LOGW(TAG, "Reconnect attempt %d", reconnect_attempts);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            mqtt_start();
            continue;
        }
        publish_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void mqtt_task(void *arg)
{
    mqtt_cmd_queue = xQueueCreate(MQTT_CMD_QUEUE_LEN, sizeof(mqtt_cmd_msg_t));
    if (mqtt_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Create mqtt cmd queue failed");
        vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
    xTaskCreate(mqtt_listen_task, "mqtt_listen", 8192, NULL, 5, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_publish", 8192, NULL, 4, NULL);
    vTaskDelete(NULL);
}
