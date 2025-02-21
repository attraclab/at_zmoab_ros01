#ifndef __SWITCH_H__
#define __SWITCH_H__

#include <Arduino.h>

// extern bool sw1_val;
// extern bool sw2_val;
// extern bool sw3_val;
// extern bool sw4_val;
extern int8_t sw_val_list[4];

void setup_switch();
bool get_switch_value(int _SW);
void switch_loop();

#endif