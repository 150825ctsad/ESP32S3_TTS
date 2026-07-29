#ifndef __RELAY_H
#define __RELAY_H

void relay_init(void);
void relay_on(uint8_t GPIO_NUM_RELAY);
void relay_off(uint8_t GPIO_NUM_RELAY);

#define  GPIO_NUM_RELAY1 42
#define  GPIO_NUM_RELAY2 41
#define  GPIO_NUM_RELAY3 40
#define  GPIO_NUM_RELAY4 39
#define  GPIO_NUM_RELAY5 38
#define  GPIO_NUM_RELAY6 37
#define  GPIO_NUM_RELAY7 36
#define  GPIO_NUM_RELAY8 35

extern uint8_t relay_state1;
extern uint8_t relay_state2;
extern uint8_t relay_state3;
extern uint8_t relay_state4;
extern uint8_t relay_state5;
extern uint8_t relay_state6;
extern uint8_t relay_state7;
extern uint8_t relay_state8;
#endif // __RELAY_H