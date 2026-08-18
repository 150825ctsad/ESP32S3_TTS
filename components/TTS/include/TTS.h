#ifndef _TTS_H_
#define _TTS_H_

#include <stdint.h>
#include "esp_err.h"

/* ---- TTS engine ---- */
esp_err_t tts_init(void);
void tts_speak_async(const char *text);

/* 设置播报完成回调（用于 MQTT 返回 ok） */
typedef void (*tts_complete_cb_t)(const char *text);
void tts_set_complete_callback(tts_complete_cb_t cb);

#endif
