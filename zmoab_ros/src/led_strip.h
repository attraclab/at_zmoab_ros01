#ifndef __LED_STRIP_H__
#define __LED_STRIP_H__

#include <Arduino.h>
#include <FastLED.h>

// extern int8_t led_strip_state;


void fillLED(uint8_t red, uint8_t green, uint8_t blue);
void fillLED(uint8_t rgb[3]);
void blink_steady(uint8_t RGB_arr[3], unsigned long on_time_ms, unsigned long off_time_ms);
void blink_heartbeat(uint8_t RGB_arr[3], unsigned long pause_time_ms);
void led_moving(uint8_t RGB_arr[3], unsigned long move_time_ms);
void rainbow_growing();
void set_led_strip_state(int8_t _state);

void setup_led_strip();
void led_strip_loop();
static void led_strip_worker_task(void *pvParameters);


#endif