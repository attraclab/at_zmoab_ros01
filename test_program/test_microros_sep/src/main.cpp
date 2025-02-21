#include <Arduino.h>
#include "ros_com.h"


void setup() {

    USBSerial.begin(115200);

    set_microros_serial_transports(USBSerial);
    delay(2000);
    init_ros_msgs();

}

void loop() {

    ros_loop();

}

