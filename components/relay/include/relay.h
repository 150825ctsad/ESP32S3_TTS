#ifndef __RELAY_H
#define __RELAY_H

#include <stdint.h>

#define RELAY_COUNT 8

extern const uint8_t relay_ctrl_pins[RELAY_COUNT];
extern const uint8_t relay_state_pins[RELAY_COUNT];
extern uint8_t box_state[RELAY_COUNT];

void relay_init(void);
void relay_state_init(void);
void relay_set(uint8_t box, uint8_t on);
void box_trigger(uint8_t box,uint8_t cmd);
uint8_t relay_read_states(void);
void relay_task(void *arg);

#endif
