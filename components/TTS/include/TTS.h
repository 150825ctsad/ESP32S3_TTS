#ifndef _TTS_H_
#define _TTS_H_

#include <stdint.h>
#include "esp_err.h"

/* ---- TTS engine ---- */
esp_err_t tts_init(void);
void tts_speak_async(const char *text);

#endif
