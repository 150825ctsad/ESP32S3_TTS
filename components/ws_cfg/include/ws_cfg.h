#ifndef __WS_CFG_H
#define __WS_CFG_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 初始化 WebSocket 会话组件
 *
 * 创建会话任务（麦克风采集 + WakeNet 唤醒 + 上行组帧 + VAD）与
 * 播放任务（下行 PCM 流式播放）。唤醒词监听立即开始。
 */
esp_err_t ws_cfg_init(void);

/**
 * @brief 保存 WebSocket 地址（mqtt_cfg 收到 /ws/ 路径时调用）
 *
 * 内部把 https:// 归一化为 wss://。
 */
esp_err_t ws_cfg_set_uri(const char *uri);

/** @brief 是否已有可用的 WebSocket 地址 */
bool ws_cfg_has_uri(void);

/**
 * @brief 云端下发 ws 后立刻发起一次 TTS 播放会话（不走麦克风上行）
 *
 * 播完后通过 ws_cfg_set_push_done_cb 回调 msgId。
 * msgId 无效（空、"string" 等）或云端 PCM 不完整时，改用本地 TTS 播 tts 文本。
 * 若当前正在语音会话中，会等到回到 IDLE 再处理。
 */
esp_err_t ws_cfg_request_push(const char *msg_id, const char *tts_text);

typedef void (*ws_cfg_push_done_cb_t)(const char *msg_id, bool ok);
void ws_cfg_set_push_done_cb(ws_cfg_push_done_cb_t cb);

/**
 * @brief 播放 voice_data 分区中的提示音 WAV（16kHz/16bit/mono）
 *
 * 阻塞播放直到结束。用于开机欢迎音、唤醒无地址提示、连接失败提示。
 */
esp_err_t ws_cfg_play_tone(void);

#endif // __WS_CFG_H
