#ifndef __GPIO_LED_H__
#define __GPIO_LED_H__

#include <Arduino.h>

void setup_led();
void IRAM_ATTR ledAgentConnected_ISR();
void set_boardLED_byId(uint8_t _id, bool flag);
void led_loop();


#endif