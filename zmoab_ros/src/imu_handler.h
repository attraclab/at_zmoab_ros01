#ifndef __IMU_H__
#define __IMU_H__

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* --- */
/* IMU */
/* --- */

extern double qw, qx, qy, qz;
extern double ax, ay, az;
extern double gx, gy, gz;
extern bool bno055_connected;

bool init_BNO055();
void setup_imu();
void reset_imu_variables();
void imu_loop();
static void imu_worker_task(void *pvParameters);

#endif