/*
Have to diable USB CDC in order to let USBSerial visible
define ARDUINO_USB_CDC_ON_BOOT on build_unflags of platformio.ini
*/

#include <Arduino.h>

void setup() {
  //USBSerial.begin();
  USBSerial.begin(115200);
  //USBSerial.println("Start"); // This cannot be printed

}

void loop() {
  USBSerial.println("Go");
  delay(1000);

}

