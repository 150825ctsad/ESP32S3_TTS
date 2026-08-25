#ifndef __BATTERY_H
#define __BATTERY_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 初始化充电检测 GPIO 与电池分压 ADC
 *
 * 引脚与分压比见板级宏：GPIO_BATTERY_ADC、GPIO_CHARGE_DET、BATTERY_DIVIDER。
 */
esp_err_t battery_init(void);

/**
 * @brief 读取充电状态与电量百分比
 *
 * @param charging  非 NULL 时写入：true=充电中
 * @param percent   非 NULL 时写入：0~100，读不到则为 -1
 */
esp_err_t battery_get(bool *charging, int *percent);

#endif
