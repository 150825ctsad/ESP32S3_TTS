#ifndef __WS_CFG_H
#define __WS_CFG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
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
 * 含 /tcm/ 的路径只用于云端 TTS 推送；其它路径用于唤醒对讲。
 * 内部把 https:// 归一化为 wss://。
 */
esp_err_t ws_cfg_set_uri(const char *uri);

/** @brief 是否已有唤醒对讲 WebSocket 地址（不含 TCM 推送通道） */
bool ws_cfg_has_uri(void);

/** @brief 是否已有云端 TTS 推送地址 */
bool ws_cfg_has_push_uri(void);

/**
 * @brief 云端下发 ws 后立刻发起一次 TTS 播放会话（不走麦克风上行）
 *
 * 播完后通过 ws_cfg_set_push_done_cb 回调 msgid、结果和原文。
 * 有 WebSocket 地址时优先走云端 PCM；无地址、或云端失败且未播出音频时，
 * 用本地 TTS 播 tts 文本。本地 TTS 播放中若到来云端语音或唤醒，会被打断。
 */
esp_err_t ws_cfg_request_push(const char *msg_id, const char *tts_text);

typedef void (*ws_cfg_push_done_cb_t)(const char *msg_id, bool ok, const char *text);
void ws_cfg_set_push_done_cb(ws_cfg_push_done_cb_t cb);

/**
 * @brief 播放 voice_data 分区中的提示音 WAV（16kHz/16bit/mono）
 *
 * 阻塞播放直到结束。用于开机欢迎音、唤醒无地址提示、连接失败提示。
 */
esp_err_t ws_cfg_play_tone(void);

/**
 * @brief 播放内存中的 WAV（16kHz/16bit/mono PCM）
 *
 * 阻塞播放直到结束。播放期间暂停 AFE。
 */
esp_err_t ws_cfg_play_wav(const uint8_t *wav, size_t wav_size);

#endif // __WS_CFG_H
