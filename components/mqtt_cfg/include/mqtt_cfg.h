#ifndef __MQTT_CFG_H
#define __MQTT_CFG_H

#include "esp_err.h"

esp_err_t mqtt_start(void);
void mqtt_task(void *pvParameters);
#endif // __MQTT_CFG_H