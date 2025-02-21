#include <Arduino.h>
#include "ros_com.h"
#include "sbus.h"
#include "motor_control.h"


void setup() {

	USBSerial.begin(115200);

	set_microros_serial_transports(USBSerial);
	delay(2000);
	init_ros_msgs();

	// create_entities();

	setup_sbus();

	setup_motor();

}

void loop() {

	ros_loop();

	motor_control_loop();

}

