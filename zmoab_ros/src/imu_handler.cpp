#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "imu_handler.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bno055_connected = false;
unsigned long last_bno055_check_stamp;

double qw = 1.0; 
double qx = 0.0;
double qy = 0.0;
double qz = 0.0;
double ax = 0.0;
double ay = 0.0;
double az = 0.0;
double gx = 0.0;
double gy = 0.0;
double gz = 0.0;

bool do_reset = false;

bool init_BNO055(){
	/// BNO055 setup//
	if (!bno.begin()){
		/* There was a problem detecting the BNO055 ... check your connections */
		return false;
	}
	delay(1000);
	bno.setExtCrystalUse(true);
	return true;
}

void reset_imu_variables(){
	qw = 1.0; 
	qx = 0.0;
	qy = 0.0;
	qz = 0.0;
	ax = 0.0;
	ay = 0.0;
	az = 0.0;
	gx = 0.0;
	gy = 0.0;
	gz = 0.0;

}

void setup_imu(){
	
	last_bno055_check_stamp = millis();
	bno055_connected = init_BNO055();

	// xTaskCreate(imu_worker_task, "imu_worker_task", 2048, NULL, 12, NULL);
}

void imu_loop(){

	/* --------------- */
	/* Read BNO055 IMU */
	/* --------------- */
	sensors_event_t orientationData , angVelocityData , linearAccelData, accelerometerData;
	//bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
	bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
	// bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
	bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	imu::Quaternion quat = bno.getQuat();
	qw = quat.w();
	qx = quat.x();
	qy = quat.y();
	qz = quat.z();

	// ax = linearAccelData.acceleration.x;
	// ay = linearAccelData.acceleration.y;
	// az = linearAccelData.acceleration.z;

	ax = accelerometerData.acceleration.x;
	ay = accelerometerData.acceleration.y;
	az = accelerometerData.acceleration.z;

	gx = angVelocityData.gyro.x;
	gy = angVelocityData.gyro.y;
	gz = angVelocityData.gyro.z;
}

static void imu_worker_task(void *pvParameters){

	while (1){

		imu_loop();
	}
}