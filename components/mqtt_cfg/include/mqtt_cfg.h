#ifndef __MQTT_CFG_H
#define __MQTT_CFG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void mqtt_task(void *pvParameters);

/** @brief MQTT 是否已连上 Broker（云服务） */
bool mqtt_is_connected(void);

#endif // __MQTT_CFG_H
