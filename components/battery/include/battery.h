#ifndef __BATTERY_H
#define __BATTERY_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#define GPIO_BATTERY_ADC        GPIO_NUM_17   /* 100k/100k 分压中点，ADC2_CH6 */
#define GPIO_CHARGE_DET         GPIO_NUM_16    /* TP4056 CHRG，低电平=充电中 */
#define GPIO_STDBY_DET          GPIO_NUM_15   /* TP4056 STDBY，低电平=已充满 */
#define CHARGE_ACTIVE_LEVEL     0
#define BATTERY_DIVIDER         2.0f
#define BATTERY_EMPTY_MV        3000
#define BATTERY_FULL_MV         4200

/**
 * @brief 初始化充电/充满检测 GPIO 与电池分压 ADC
 */
esp_err_t battery_init(void);

/**
 * @brief 读取充电状态与电量百分比
 *
 * @param charging  非 NULL 时写入：true=充电中
 * @param percent   非 NULL 时写入：0~100，读不到则为 -1
 */
esp_err_t battery_get(bool *charging, int *percent);

/**
 * @brief 读取充电状态与电量百分比，并区分已充满
 *
 * @param charging  非 NULL 时写入：true=充电中
 * @param full      非 NULL 时写入：true=已充满（未插电时 false）
 * @param percent   非 NULL 时写入：0~100，读不到则为 -1
 */
esp_err_t battery_get_state(bool *charging, bool *full, int *percent);

void battery_task(void *arg);

#endif
