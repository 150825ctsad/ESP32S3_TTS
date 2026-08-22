#ifndef _TTS_H_
#define _TTS_H_

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief 从 tts_data 分区加载中文语音库，创建合成任务
 */
esp_err_t tts_init(void);

/** @brief 语音库是否可用 */
bool tts_ready(void);

/**
 * @brief 阻塞合成并播放中文文本（16 kHz PCM）
 *
 * 在独立 tts_task 中执行，避免占用 session 栈。
 * 可被 tts_abort() 打断，返回 ESP_FAIL。
 */
esp_err_t tts_play(const char *text);

/** @brief 打断正在进行的本地 TTS，让出喇叭给 WebSocket 语音 */
void tts_abort(void);

/** @brief 本地 TTS 是否正在合成/播放 */
bool tts_busy(void);

#endif
