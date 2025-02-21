#include <Arduino.h>
#include <ModbusMaster.h>

#define MAX485_DE  2
#define MODBUS_RX_PIN 44
#define MODBUS_TX_PIN 43

// Common
#define CONTROL_REG 0x200E
#define OPR_MODE  0x200D
#define L_ACL_TIME  0x2080
#define R_ACL_TIME  0x2081
#define L_DCL_TIME  0x2082
#define R_DCL_TIME  0x2083

// Velocity control
#define L_CMD_RPM  0x2088
#define R_CMD_RPM  0x2089
#define L_FB_RPM  0x20AB
#define R_FB_RPM  0x20AC

// Position control
#define POS_CONTROL_TYPE  0x200F

#define L_MAX_RPM_POS  0x208E
#define R_MAX_RPM_POS  0x208F

#define L_CMD_REL_POS_HI  0x208A
#define L_CMD_REL_POS_LO  0x208B
#define R_CMD_REL_POS_HI  0x208C
#define R_CMD_REL_POS_LO  0x208D

#define L_FB_POS_HI  0x20A7
#define L_FB_POS_LO  0x20A8
#define R_FB_POS_HI  0x20A9
#define R_FB_POS_LO  0x20AA

// Control command
#define EMER_STOP  0x05
#define ALRM_CLR 0x06
#define DOWN_TIME 0x07
#define ENABLE 0x08
#define POS_SYNC 0x10
#define POS_L_START 0x11
#define POS_R_START 0x12

uint8_t result;

void preTransmission(){
  digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
  digitalWrite(MAX485_DE, 0);
}

ModbusMaster node;

void setup() {

  USBSerial.begin();

  Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  delay(1000);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  result = node.writeSingleRegister(CONTROL_REG, DOWN_TIME);
  USBSerial.print("disable motor ");
  USBSerial.println(result);
  delay(2000);
  result = node.writeSingleRegister(OPR_MODE, 3);
  USBSerial.print("set speed mode ");
  USBSerial.println(result);
  result = node.writeSingleRegister(CONTROL_REG, ENABLE);
  USBSerial.print("enable motor ");
  USBSerial.println(result);
  node.writeSingleRegister(L_CMD_RPM, 0x0032);

}

void loop() {

 

}

