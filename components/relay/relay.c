#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "relay.h"
#include "driver/gpio.h"

#define BOX_TRIGGER_TIME_MS 125

const uint8_t relay_ctrl_pins[RELAY_COUNT]  = {42, 41, 40, 39, 38, 37, 36, 35};
const uint8_t relay_state_pins[RELAY_COUNT] = { 4,  5,  6,  7, 15, 16, 17, 18};
uint8_t box_state[RELAY_COUNT] = {0};

/* ---- Init control pins (output, all off) ---- */
void relay_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < RELAY_COUNT; i++)
        mask |= (1ULL << relay_ctrl_pins[i]);

    gpio_config_t conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = mask,
    };
    gpio_config(&conf);

    for (int i = 0; i < RELAY_COUNT; i++)
        relay_set(i + 1, 0);
}

/* ---- Init state feedback pins (input) ---- */
void relay_state_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < RELAY_COUNT; i++)
        mask |= (1ULL << relay_state_pins[i]);

    gpio_config_t conf = {
        .mode         = GPIO_MODE_INPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = mask,
    };
    gpio_config(&conf);
}

/* ---- Control one box (1-indexed) ---- */
void relay_set(uint8_t box,uint8_t on)
{
    if(box <1 || box>RELAY_COUNT)
        return;

    uint8_t idx=box-1; 
    gpio_set_level( relay_ctrl_pins[idx], on ? 1:0 );
}

void box_trigger(uint8_t box,uint8_t cmd)
{
    if(box < 1 || box > RELAY_COUNT)
        return;

    uint8_t idx = box-1;
    gpio_set_level(relay_ctrl_pins[idx],cmd);
    vTaskDelay(pdMS_TO_TICKS(BOX_TRIGGER_TIME_MS));
    gpio_set_level(relay_ctrl_pins[idx],0);
}

/* ---- Read all 8 state pins, return bitmap ---- */
uint8_t relay_read_states(void)
{
    uint8_t bitmap = 0;
    for (int i = 0; i < RELAY_COUNT; i++) {
        if (gpio_get_level(relay_state_pins[i]))
            bitmap |= (1 << i);
    }
    return bitmap;
}

/* ---- FreeRTOS task: poll state pins, update box_state[] ---- */
void relay_task(void *arg)
{
    while (1) {
        uint8_t states = relay_read_states();
        for (int i = 0; i < RELAY_COUNT; i++)
            box_state[i] = (states >> i) & 1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
