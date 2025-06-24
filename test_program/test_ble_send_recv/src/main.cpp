#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/// Data Streaming ///
int cart_mode = 0;
int cart_mode_cmd = 0;

// Data receiving ///
float x = 0.0;
float y = 0.0;
int pwm1_cmd = 1500;
int pwm2_cmd = 1500;
int motorValue = 1;

/// BLE setup ///
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long last_send_stamp = millis();

unsigned long log_stamp = millis();

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
    cart_mode_cmd = data.substring(idx5 + 1).toInt();
    cart_mode = cart_mode_cmd;
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
      USBSerial.println("BLE device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      USBSerial.println("BLE device disconnected, restarting advertising");
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
    }
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-IMU");
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
  USBSerial.println("BLE advertising started");
}



void setup() {
  USBSerial.begin(115200);

  /// BLE ///
  setupBLE();
}

void loop() {

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
    USBSerial.println("Restarted advertising after disconnect");
    oldDeviceConnected = deviceConnected;
  }

  // Track device connection state
  if (deviceConnected && !oldDeviceConnected) {
    USBSerial.println("Device connected");
    oldDeviceConnected = deviceConnected;
  }

  if ((millis() - log_stamp) > 50){
    USBSerial.print(log_stamp);
    USBSerial.printf(" x: %.2f ", x);
    USBSerial.printf("y: %.2f ", y);
    USBSerial.printf("pwm1: %d ", pwm1_cmd);
    USBSerial.printf("pwm2: %d ", pwm2_cmd);
    USBSerial.printf("motor: %d ", motorValue);
    USBSerial.printf("cart_mode_cmd: %d\n", cart_mode_cmd);

    log_stamp = millis();
  }

  delay(10);
}
