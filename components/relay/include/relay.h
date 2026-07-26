#ifndef __RELAY_H
#define __RELAY_H

void relay_init(void);
void relay_on(void);
void relay_off(void);

extern uint8_t relay_state;
#endif // __RELAY_H