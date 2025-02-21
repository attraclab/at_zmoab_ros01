#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool bno055_connected = false;
unsigned long last_bno055_check_stamp;

bool init_BNO055(){
  /// BNO055 setup//
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    USBSerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  return true;
}

void setup() {

  USBSerial.begin(115200);

  last_bno055_check_stamp = millis();
  bno055_connected = init_BNO055();

}

void loop() {

  if (bno055_connected){
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    USBSerial.print("X: ");
    USBSerial.print(event.orientation.x, 4);
    USBSerial.print("\tY: ");
    USBSerial.print(event.orientation.y, 4);
    USBSerial.print("\tZ: ");
    USBSerial.print(event.orientation.z, 4);
    USBSerial.println("");
  } else {
    if ((millis() - last_bno055_check_stamp) > 3000){
      last_bno055_check_stamp = millis();
      bno055_connected = init_BNO055();
    }
  }
  

}

