#include <stdio.h>
#include "relay.h"
#include "driver/gpio.h"

#define  GPIO_NUM_RELAY 42

uint8_t relay_state = 0;
void relay_init(void)
{
    // 配置GPIO引脚
    gpio_config_t conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << GPIO_NUM_RELAY),
       };
    gpio_config(&conf);
    
    // 初始状态设为关闭
    relay_off();
}

void relay_on(void)
{
    // 设置引脚为高电平（打开继电器）
    gpio_set_level(GPIO_NUM_RELAY, 1);
    relay_state = 1;
}

void relay_off(void)
{
    // 设置引脚为低电平（关闭继电器）
    gpio_set_level(GPIO_NUM_RELAY, 0);
    relay_state = 0;
}
