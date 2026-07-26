#include <stdio.h>
#include "wifi_cfg.h"
#include "mqtt_cfg.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/semphr.h"
#include <inttypes.h>

#define SSID "SUMMER"
#define PASSWORD "00000000"

void wifi_event_handler(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data) 
{
    // Handle WiFi events here
    ESP_LOGI("WIFI_EVENT", "Event received: base=%s, id=%" PRId32, event_base, event_id);
    if(event_base == WIFI_EVENT) {
        switch(event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                ESP_LOGI("WIFI_EVENT", "Station started");
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI("WIFI_EVENT", "Station connected to AP");
                break;
            case WIFI_EVENT_STA_STOP:
                ESP_LOGI("WIFI_EVENT", "Station stopped");
                break;
            // Handle other WiFi events as needed
        }
    }
    else if(event_base == IP_EVENT) {
        switch(event_id) {
            case IP_EVENT_STA_GOT_IP:
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI("IP_EVENT", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL, 1);

                break;
            // Handle other IP events as needed
        }
    }
}
void wifi_init_sta(void) 
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    wifi_config_t wifi_config = {
        .sta.ssid = SSID,
        .sta.password = PASSWORD,
        .sta.threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .sta.pmf_cfg.capable = true,
        .sta.pmf_cfg.required = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}