#ifndef __BLE_INTERFACE_H__
#define __BLE_INTERFACE_H__

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

void onReceiveBLE(std::string bleData);
void setupBLE();
void ble_loop();

#endif