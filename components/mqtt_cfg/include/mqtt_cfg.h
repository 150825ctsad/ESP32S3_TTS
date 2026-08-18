#ifndef __MQTT_CFG_H
#define __MQTT_CFG_H

#include <stdint.h>
#include <stddef.h>

static void mqtt_start(void);
void mqtt_task(void *pvParameters);

/* 发布音频数据到 audio/<mac> 主题（Base64 编码 PCM） */
void mqtt_publish_audio(const char *b64_data, size_t b64_len);

#endif // __MQTT_CFG_H