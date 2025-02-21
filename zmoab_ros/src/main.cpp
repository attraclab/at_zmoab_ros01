/*
	ZMOAB Micro-ROS on ESP32-S3-WROOM-1 
	developed by Rasheed Kittinanthapanya
	belong under ATTRACLAB

*/

#include <Arduino.h>

#include "ros_com.h"
#include "sbus.h"
#include "motor_control.h"
#include "imu_handler.h"
#include "gpio_led.h"
#include "pwm_handler.h"
#include "switch_handler.h"
#include "led_strip.h"


/////////////////////
/// Arduino Setup ///
/////////////////////
void setup() {

	/* --------- */
	/* USB setup */ 
	/* --------- */
	USBSerial.begin(115200);

	/* --------------- */
	/* Micro ROS setup */ 
	/* --------------- */
	set_microros_serial_transports(USBSerial);
	delay(2000);

	init_ros_msgs();

	setup_sbus();
	setup_motor();
	setup_imu();
	setup_led();
	setup_servo();
	setup_switch();
	setup_led_strip();
}

////////////////////
/// Arduino Loop ///
////////////////////
void loop() {

	ros_loop();

	motor_control_loop();

	imu_loop();

	led_loop();

	switch_loop();

	led_strip_loop();

}

