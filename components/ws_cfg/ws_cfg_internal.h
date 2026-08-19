#ifndef __WS_CFG_INTERNAL_H
#define __WS_CFG_INTERNAL_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 会话事件位（ws_cfg transport → voice_session） */
#define WS_EVT_CONNECTED    (1 << 0)   /* WS 连接成功 */
#define WS_EVT_DISCONNECTED (1 << 1)   /* WS 断开（非主动） */
#define WS_EVT_TTS_DONE     (1 << 2)   /* 收到 {"type":"done"} */
#define WS_EVT_TTS_ERROR    (1 << 3)   /* 收到 {"type":"error"} */
#define SESS_EVT_PUSH       (1 << 10)  /* MQTT 下发 ws，请求立刻连云端播 TTS */

/**
 * @brief 注入会话事件组与下行音频缓冲（voice_session 初始化时调用）
 */
void ws_cfg_attach(EventGroupHandle_t evt, ringbuf_handle_t ring);

/* ---------- transport API（仅 voice_session 使用） ---------- */

/** @brief 用已保存的地址建立 WS 连接（异步，成功由 WS_EVT_CONNECTED 通知） */
esp_err_t ws_cfg_connect(void);

/** @brief 会话结束：断开并释放客户端（主动断开不产生 WS_EVT_DISCONNECTED 处理需求） */
esp_err_t ws_cfg_disconnect(void);

/** @brief 当前是否已连接 */
bool ws_cfg_is_connected(void);

/** @brief 发送 JSON 文本帧（仅 session_task 上下文调用） */
esp_err_t ws_cfg_send_text(const char *json);

/** @brief 发送 PCM16 二进制帧（仅 session_task 上下文调用） */
esp_err_t ws_cfg_send_pcm(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif // __WS_CFG_INTERNAL_H
