#include "ble_interface.h"
#include "sbus.h"
#include "motor_control.h"
#include "pwm_handler.h"

/// Data Streaming ///
int ble_cart_mode = 0;
int ble_cart_mode_cmd = 0;
int prev_ble_cart_mode_cmd = 0;

// Data receiving ///
float x = 0.0;
float y = 0.0;
int pwm1_cmd = 1500;
int pwm2_cmd = 1500;
int motorValue = 1;
int prevMotorValue = 1;
int prev_pwm1_cmd = pwm1_cmd;
int prev_pwm2_cmd = pwm2_cmd;

/// BLE setup ///
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long last_send_stamp = millis();

unsigned long loop_update_stamp = millis();

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // TX Notify + RX Write + Read

void onReceiveBLE(std::string bleData) {

  String data = String(bleData.c_str());
  // float x = 0.0, y = 0.0;
  // int speedLeft = 0, speedRight = 0, mode = 0;

  // Split the string by commas
  int idx1 = data.indexOf(',');
  int idx2 = data.indexOf(',', idx1 + 1);
  int idx3 = data.indexOf(',', idx2 + 1);
  int idx4 = data.indexOf(',', idx3 + 1);
  int idx5 = data.indexOf(',', idx4 + 1);

  // if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3) {
  if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3 && idx5 > idx4) {
    x = data.substring(0, idx1).toFloat();
    y = data.substring(idx1 + 1, idx2).toFloat();
    pwm1_cmd = data.substring(idx2 + 1, idx3).toInt();
    pwm2_cmd = data.substring(idx3 + 1, idx4).toInt();
    motorValue = data.substring(idx4 + 1, idx5).toInt();
    ble_cart_mode_cmd = data.substring(idx5 + 1).toInt();
    ble_cart_mode = ble_cart_mode_cmd;
    // Debug output
    // Serial.printf("x: %.2f, y: %.2f\n", x, y);
    // Serial.printf("speedLeft: %d, speedRight: %d\n", speedLeft, speedRight);
    // Serial.printf("mode: %d\n", mode);
  // } else {
  //   USBSerial.println("Invalid data format!");
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      // USBSerial.println("BLE device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;

      // USBSerial.println("BLE device disconnected, restarting advertising");
      delay(1000); // give some time before restarting advertising
      pServer->startAdvertising();
      pServer->getAdvertising()->setMinPreferred(300);  // set desired MTU
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      // String value_string = String(value.c_str());
      //USBSerial.print("Received: ");
      //USBSerial.println(value_string);
      onReceiveBLE(value);
      last_recv_rpm_cmd_stamp = millis();
    }
  }
};

void setupBLE() {
  BLEDevice::init("AT_ZMOAB_ROS");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  pServer->getAdvertising()->setMinPreferred(300);  // set desired MTU
  // USBSerial.println("BLE advertising started");
}

void ble_loop() {

  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%d\n", cart_mode);

  /// send data via BLE every 1 second
  if (((millis() - last_send_stamp) > 1000) && deviceConnected) {
    pCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pCharacteristic->notify();
    last_send_stamp = millis();
  }

  // Restart advertising if device was disconnected
  if (!deviceConnected && oldDeviceConnected) {
    
    delay(1000);
    pServer->startAdvertising();
    pServer->getAdvertising()->setMinPreferred(300);  // set desired MTU
    // USBSerial.println("Restarted advertising after disconnect");
    oldDeviceConnected = deviceConnected;
  }

  // Track device connection state
  if (deviceConnected && !oldDeviceConnected) {
    // USBSerial.println("Device connected");
    oldDeviceConnected = deviceConnected;
  }

  // if ((millis() - loop_update_stamp) > 10){
  if (deviceConnected) {
    // USBSerial.print(loop_update_stamp);
    // USBSerial.printf(" x: %.2f ", x);
    // USBSerial.printf("y: %.2f ", y);
    // USBSerial.printf("pwm1: %d ", pwm1_cmd);
    // USBSerial.printf("pwm2: %d ", pwm2_cmd);
    // USBSerial.printf("motor: %d ", motorValue);
    // USBSerial.printf("cart_mode_cmd: %d\n", ble_cart_mode);

    /// Cart mode change ///
    if (ble_cart_mode_cmd != prev_ble_cart_mode_cmd){
      set_cart_mode((int8_t)ble_cart_mode_cmd);
      prev_ble_cart_mode_cmd = ble_cart_mode;
    } else if (prev_ble_cart_mode_cmd != cart_mode){
      // in case cart mode was changed from RC,
      // so we update it here too.
      prev_ble_cart_mode_cmd = cart_mode;
      ble_cart_mode_cmd = cart_mode;
      ble_cart_mode = cart_mode;
    }
    
    /// PWM servos ///
    if ((prev_pwm1_cmd != pwm1_cmd) || (prev_pwm2_cmd != pwm2_cmd)){
      write_pwm((int16_t)pwm1_cmd, (int16_t)pwm2_cmd);
      prev_pwm2_cmd = pwm1_cmd;
      prev_pwm2_cmd = pwm2_cmd;
    }
    
    /// Disable motor ///
    if ((motorValue != prevMotorValue) && (motorValue == 1)){
      set_enable_motor();
      prevMotorValue = motorValue;
    } else if ((motorValue != prevMotorValue) && (motorValue == 0)){
      set_disable_motor();
      prevMotorValue = motorValue;
    }

    /// Motor Control ///
    if (ble_cart_mode_cmd == 2){
      // last_recv_rpm_cmd_stamp = millis();
      float rpm_cmd[2];
      channelMixing_by_Joystick(x,-y, rpm_cmd);
      set_rpm_cmd(rpm_cmd[0], rpm_cmd[1]);
    }
    
    loop_update_stamp = millis();
  }

  //delay(10);
}
