#include <Arduino.h>

// #define FASTLED_USES_ESP32S3_I2S

#include <FastLED.h>
// #include "fl/assert.h"

#define FAST_LED_PIN 14
#define NUM_LEDS 144

// const bool gUseFastLEDApi = true;  // Set this to false to use the raw driver.
// fl::InternalI2SDriver *i2s_driver = nullptr;

CRGB leds[NUM_LEDS];

uint8_t rgb_arr[3] = {0, 0, 255};

unsigned long last_fill_stamp;
bool on_state;
uint8_t hb_state;
bool move_state_init;
uint16_t led_counter;

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

uint8_t hue = 0;
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

int count = 0;
unsigned long last_switch_led_stamp = millis();


void setup() {

	FastLED.addLeds<NEOPIXEL, FAST_LED_PIN>(leds, NUM_LEDS);
	FastLED.setBrightness(50);

	last_fill_stamp = millis();
	on_state = false;
	hb_state = 0;
	led_counter = 0;
	move_state_init = false;

	last_switch_led_stamp = millis();

}

void loop() {

	if ((millis() - last_switch_led_stamp) > 3000){
		count += 1;
		last_switch_led_stamp = millis();
	} 

	if (count > 4){
		count = 0;
	}

	if (count == 0){
		fillLED(rgb_arr);
	} else if (count == 1){
		rgb_arr[0] = 255;
		rgb_arr[1] = 0;
		rgb_arr[2] = 0;
		blink_steady(rgb_arr, 700, 700);
	} else if (count == 2){
		rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 0;
		blink_heartbeat(rgb_arr, 700);
	} else if (count == 3) {
		rgb_arr[0] = 0;
		rgb_arr[1] = 255;
		rgb_arr[2] = 255;
		led_moving(rgb_arr, 40);
	} else if (count == 4){
		rainbow_growing();
	}




	
	

}

