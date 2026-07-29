#include <stdio.h>
#include "relay.h"
#include "driver/gpio.h"

uint8_t relay_state1 = 0;
uint8_t relay_state2 = 0;
uint8_t relay_state3 = 0;
uint8_t relay_state4 = 0;
uint8_t relay_state5 = 0;
uint8_t relay_state6 = 0;
uint8_t relay_state7 = 0;
uint8_t relay_state8 = 0;

void relay_init(void)
{
    // 配置GPIO引脚
    gpio_config_t conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << GPIO_NUM_RELAY1) | (1ULL << GPIO_NUM_RELAY2) | (1ULL << GPIO_NUM_RELAY3) | (1ULL << GPIO_NUM_RELAY4) | (1ULL << GPIO_NUM_RELAY5) | (1ULL << GPIO_NUM_RELAY6) | (1ULL << GPIO_NUM_RELAY7) | (1ULL << GPIO_NUM_RELAY8),
       };
    gpio_config(&conf);
    
    // 初始状态设为关闭
    relay_off(GPIO_NUM_RELAY1);
    relay_off(GPIO_NUM_RELAY2);
    relay_off(GPIO_NUM_RELAY3);
    relay_off(GPIO_NUM_RELAY4);
    relay_off(GPIO_NUM_RELAY5);
    relay_off(GPIO_NUM_RELAY6);
    relay_off(GPIO_NUM_RELAY7);
    relay_off(GPIO_NUM_RELAY8);
}

void relay_on(uint8_t GPIO_NUM_RELAY)
{
    // 设置引脚为高电平（打开继电器）
    gpio_set_level(GPIO_NUM_RELAY, 1);
    relay_state1 = 1;
}

void relay_off(uint8_t GPIO_NUM_RELAY)
{
    // 设置引脚为低电平（关闭继电器）
    gpio_set_level(GPIO_NUM_RELAY, 0);
    relay_state1 = 0;
}
