#include <Arduino.h>
#include <ModbusMaster.h>
#include <ZLAC8015D.h>

////////////////////////////////
/// ZLAC8015D & ModbusMaster ///
////////////////////////////////
#define MAX485_DE  2
#define MODBUS_RX_PIN 44
#define MODBUS_TX_PIN 43

void preTransmission(){
  digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
  digitalWrite(MAX485_DE, 0);
}

ModbusMaster node;
ZLAC8015D driver;
uint8_t res;
int16_t rpmL;
int16_t rpmR;
int16_t rpmFB[2];

void setup() {

  USBSerial.begin();

  Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  delay(1000);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  driver.set_modbus(&node);
  bool done = false;
  
  while (! done) {
    res = driver.disable_motor();
    USBSerial.print("disable motor ");
    USBSerial.println(res);
    if (res == 0) {
      done = true;
    }
  }
  res = driver.set_mode(3);
  USBSerial.print("set mode ");
  USBSerial.println(res);

  
  res = driver.enable_motor();
  USBSerial.print("enable motor ");
  USBSerial.println(res);

  res = driver.set_accel_time(0, 0);
  USBSerial.print("set accel ");
  USBSerial.println(res);

  res = driver.set_decel_time(0, 0);
  USBSerial.print("set decel ");
  USBSerial.println(res);
  delay(1000);
}

void loop() {

  rpmL = 30;
  rpmR = 50;

  res = driver.set_rpm(rpmL, rpmR);
  res = driver.get_rpm(rpmFB);

  USBSerial.print("rpmL_cmd: ");
  USBSerial.print(rpmL);
  USBSerial.print(" rpmR_cmd: ");
  USBSerial.print(rpmR);
  USBSerial.print(" rpmL_fb: ");
  USBSerial.print(rpmFB[0]);
  USBSerial.print(" rpmR_fb: ");
  USBSerial.println(rpmFB[1]);

  delay(100);
}

