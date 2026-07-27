#ifndef _TTS_H_
#define _TTS_H_

#define URAT_BUF_LEN 1024
void uartTask(void *arg);
void tts_speak_async(const char *text);

#endif
