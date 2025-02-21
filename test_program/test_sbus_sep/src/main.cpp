#include <Arduino.h>
#include "ros_com.h"
#include "sbus.h"


void setup() {

    USBSerial.begin(115200);

    set_microros_serial_transports(USBSerial);
    delay(2000);
    init_ros_msgs();

    setup_sbus();

}

void loop() {

    ros_loop();

}

