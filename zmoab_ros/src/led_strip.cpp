#include <Arduino.h>
#include <FastLED.h>
#include "led_strip.h"

#define FAST_LED_PIN 14
#define NUM_LEDS 144

int8_t led_strip_state = 0;

CRGB leds[NUM_LEDS];

uint8_t rgb_arr[3] = {0, 0, 255};

unsigned long last_fill_stamp;
bool on_state;
uint8_t hb_state;
bool move_state_init;
uint16_t led_counter;

uint8_t hue = 0;

void fillLED(uint8_t red, uint8_t green, uint8_t blue) {
	for (int i = 0; i < NUM_LEDS; i++){
		leds[i] = CRGB(red, green, blue);
	}
	FastLED.show();
}

void fillLED(uint8_t rgb[3]) {
	for (int i = 0; i < NUM_LEDS; i++){
		leds[i] = CRGB(rgb[0], rgb[1], rgb[2]);
	}
	FastLED.show();
}

void blink_steady(uint8_t RGB_arr[3], unsigned long on_time_ms, unsigned long off_time_ms) {
	/*
		Blink steady with constant frequency with on time and off time
	*/

	if (! on_state) {
		if ((millis() - last_fill_stamp) > off_time_ms)  {
			fillLED(RGB_arr[0], RGB_arr[1], RGB_arr[2]);
			last_fill_stamp = millis();
			on_state = true;
		}
	}
	else {
		if ((millis() - last_fill_stamp) > on_time_ms) {
			fillLED(0, 0, 0);
			last_fill_stamp = millis();
			on_state = false;
		}
	}
}

void blink_heartbeat(uint8_t RGB_arr[3], unsigned long pause_time_ms) {
	/*
		Blink two times quickly then pause for a while

		hb_state 0 pulse up first
		hb_state 1 pulse down first
		hb_state 2 pulse up second
		hb_state 3 pulse down second
		hb_state 4 pulse down long res
	*/
	if ((hb_state == 0) or (hb_state == 2)) {
		if ((millis() - last_fill_stamp) > 100) {
			fillLED(RGB_arr[0], RGB_arr[1], RGB_arr[2]);
			last_fill_stamp = millis();
			if (hb_state == 0) {
				hb_state = 1;
			} else if (hb_state == 2) {
				hb_state = 3;
			}
		}
	} else if ((hb_state == 1) or (hb_state == 3)) {
		if ((millis() - last_fill_stamp) > 100) {
			fillLED(0, 0, 0);
			last_fill_stamp = millis();
			if (hb_state == 1) {
				hb_state = 2;
			} else if (hb_state == 3) {
				hb_state = 4;
			}
		}
	} else if (hb_state == 4) {
		if ((millis() - last_fill_stamp) > pause_time_ms) {
			fillLED(0, 0, 0);
			last_fill_stamp = millis();
			hb_state = 0;
		}
	}
}

void led_moving(uint8_t RGB_arr[3], unsigned long move_time_ms) {
  /*

  */
	if (! move_state_init) {
		fillLED(0, 0, 0);
		move_state_init = true;
		led_counter = 0;
	}
	else {
		if ((millis() - last_fill_stamp) > move_time_ms) {
			leds[led_counter] = CRGB(RGB_arr[0], RGB_arr[1], RGB_arr[2]);
			FastLED.show();
			led_counter += 1;
			last_fill_stamp = millis();
			if (led_counter >= NUM_LEDS){
				move_state_init = false;
			}
		}
	}
}

void rainbow_growing() {
	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CHSV(hue + (i * 10), 255, 255);
	}

	EVERY_N_MILLISECONDS(5) {
    hue++;
	}

	if (hue > 255) {
		hue = 0;
	}

	FastLED.show();

}

void set_led_strip_state(int8_t _state){
	led_strip_state = _state;

	last_fill_stamp = millis();
	on_state = false;
	hb_state = 0;
	led_counter = 0;
	move_state_init = false;
}

void setup_led_strip(){

    FastLED.addLeds<NEOPIXEL, FAST_LED_PIN>(leds, NUM_LEDS);
	FastLED.setBrightness(50);

	last_fill_stamp = millis();
	on_state = false;
	hb_state = 0;
	led_counter = 0;
	move_state_init = false;

	// xTaskCreate(led_strip_worker_task, "led_strip_worker_task", 2048, NULL, 12, NULL);


}

void led_strip_loop(){

	/// Solid color ///
    if (led_strip_state == 0){
        rgb_arr[0] = 0;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		fillLED(rgb_arr);
	} else if (led_strip_state == 1){
        rgb_arr[0] = 255;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		fillLED(rgb_arr);
    } else if (led_strip_state == 2){
        rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 0;
		fillLED(rgb_arr);
    } else if (led_strip_state == 3){
        rgb_arr[0] = 0;
		rgb_arr[1] = 0;
		rgb_arr[2] = 255;
		fillLED(rgb_arr);
    } 

	/// Blink steady ///
	else if (led_strip_state == 11){
        rgb_arr[0] = 255;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		blink_steady(rgb_arr, 200, 200);
    } else if (led_strip_state == 12){
        rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 0;
		blink_steady(rgb_arr, 200, 200);
    } else if (led_strip_state == 13){
        rgb_arr[0] = 0;
		rgb_arr[1] = 0;
		rgb_arr[2] = 255;
		blink_steady(rgb_arr, 200, 200);
    } 
	

	else if (led_strip_state == 21){
        rgb_arr[0] = 255;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		blink_heartbeat(rgb_arr, 700);
    } else if (led_strip_state == 22){
        rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 0;
		blink_heartbeat(rgb_arr, 700);
    } else if (led_strip_state == 23){
        rgb_arr[0] = 0;
		rgb_arr[1] = 0;
		rgb_arr[2] = 255;
		blink_heartbeat(rgb_arr, 700);
    } 
	
	
	/// Led moving ///
	else if (led_strip_state == 31){
        rgb_arr[0] = 255;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		led_moving(rgb_arr, 10);
    } else if (led_strip_state == 32){
        rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 0;
		led_moving(rgb_arr, 10);
    } else if (led_strip_state == 33){
        rgb_arr[0] = 0;
		rgb_arr[1] = 0;
		rgb_arr[2] = 255;
		led_moving(rgb_arr, 10);
    }
	
	/// Rainbox ///
	else if (led_strip_state == 100){
        rainbow_growing();
    }

}

static void led_strip_worker_task(void *pvParameters){

	while (1){

		led_strip_loop();
	}
}