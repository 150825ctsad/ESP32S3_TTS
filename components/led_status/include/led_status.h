#ifndef __LED_STATUS_H
#define __LED_STATUS_H

/**
 * @brief 红/绿双色灯：上电即红绿闪，wifi_init 结束后按网络/云状态自动切换
 */
void led_status_init(void);

/** @brief 启动阶段结束，之后按配网 / WiFi / MQTT 显示 */
void led_status_boot_done(void);

#endif
