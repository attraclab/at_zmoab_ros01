#include <Arduino.h>
#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include "motor_control.h"
#include "sbus.h"

/* ------------------------ */
/* ZLAC8015D & ModbusMaster */
/* ------------------------ */
#define MAX485_DE  2
#define MODBUS_RX_PIN 43 //44
#define MODBUS_TX_PIN 44 //43

ModbusMaster ModbusNode;
ZLAC8015D driver;

uint8_t res;

int16_t rpmL;
int16_t rpmR;
float rpm[2];
float prev_y = 0.0;
float max_rpm = 200.0;
float rpmDB = 5.0;


unsigned long last_fault_code_stamp;
bool got_faultCode = false;
bool prev_got_faultCode = false;

int16_t _rpmFB[2];
int32_t _encoderFB[2];
int16_t _faultCode[2];

int16_t rpmFB[2] = {0, 0};
int32_t encoderFB[2] =  {0, 0};
int16_t faultCode[2] =  {0, 0};
unsigned long last_recv_rpm_cmd_stamp = millis();


void preTransmission(){
	digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
	digitalWrite(MAX485_DE, 0);
}

void channelMixing(uint16_t str_ch, uint16_t thr_ch, float _rpm[2]){
	
	float y;
	float x;

	if (thr_ch >= sbus_max_db){
		y = (float)map(thr_ch, sbus_max_db, sbus_max, 0.0, 100.0);
	} else if (thr_ch <= sbus_min_db){
		y = (float)map(thr_ch, sbus_min, sbus_min_db, -100.0, 0.0);
	} else {
		y = 0.0;
	}

	if (str_ch >= sbus_max_db){
		x = (float)map(str_ch, sbus_max_db, sbus_max, 0.0, 100.0);
	} else if (str_ch <= sbus_min_db){
		x = (float)map(str_ch, sbus_min, sbus_min_db, -100.0, 0.0);
	} else {
		x = 0.0;
	}
	
	float left, right;
	left = y + x;
	right = y - x;
	float diff = abs(x) - abs(y);

	if (left < 0.0){
		left = left - abs(diff);
	} else {
		left = left + abs(diff);
	}

	if (right < 0.0){
		right = right - abs(diff);
	} else {
		right = right + abs(diff);
	}

	if (prev_y < 0.0){
		float swap;
		swap = left;
		left = right;
		right = swap;
	}

	prev_y = y;

	_rpm[0] = map(left, -200.0, 200.0, -max_rpm, max_rpm);
	_rpm[1] = map(right, -200.0, 200.0, -max_rpm, max_rpm);

	if ((_rpm[0] < rpmDB) && (_rpm[0] > -rpmDB)){
		_rpm[0] = 0.0;
	}
	if ((_rpm[1] < rpmDB) && (_rpm[1] > -rpmDB)){
		_rpm[1] = 0.0;
	}
}

void setup_motor(){
	Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
	pinMode(MAX485_DE, OUTPUT);
	digitalWrite(MAX485_DE, 0);
	delay(1000);
	ModbusNode.begin(1, Serial);
	ModbusNode.preTransmission(preTransmission);
	ModbusNode.postTransmission(postTransmission);
	driver.set_modbus(&ModbusNode);
	bool done = false;

	while (! done) {
		res = driver.disable_motor();
		if (res == 0) {
			done = true;
		}
	}
	res = driver.set_mode(3);
	res = driver.enable_motor();
	res = driver.set_accel_time(0, 0);
	res = driver.set_decel_time(0, 0);

	delay(1000);
	last_fault_code_stamp = millis();
}
void set_enable_motor(uint8_t flag){
	if (flag == 1){
			res = driver.disable_motor();
	} else {
			res = driver.enable_motor();
	}	
}

void set_rpm_cmd(int16_t _rpmL, int16_t _rpmR){

	rpmL = _rpmL;
	rpmR = _rpmR;

}

void motor_control_loop(){
	if (cart_mode == 2){
		if ((millis() - last_recv_rpm_cmd_stamp) >= 1000){
			rpmL = 0;
			rpmR = 0;
			res = driver.set_rpm(rpmL, rpmR);
		} else {
			res = driver.set_rpm(rpmL, rpmR);
		}
				
	} else if (cart_mode == 1){
		memcpy(sbus_ch, ch, sizeof(ch));
		channelMixing(sbus_ch[0], sbus_ch[1], rpm);
		rpmL = (int16_t)rpm[0];
		rpmR = (int16_t)rpm[1];

		res = driver.set_rpm(rpmL, rpmR);
				
	} else {
		rpmL = 0;
		rpmR = 0;
		res = driver.set_rpm(rpmL, rpmR);
	}

	res = driver.get_rpm(_rpmFB);
	res = driver.get_encoder_count(_encoderFB);

	rpmFB[0] = _rpmFB[0];
	rpmFB[1] = _rpmFB[1];

	encoderFB[0] = _encoderFB[0];
	encoderFB[1] = _encoderFB[1];


	/* -------------------- */
	/* Check ESC Fault Code */
	/* -------------------- */
	// if ((millis() - last_fault_code_stamp) >= 0){
	// 	res = driver.get_fault_code(_faultCode);

	// 	faultCode[0] = _faultCode[0];
	// 	faultCode[1] = _faultCode[1];
		
	// 	last_fault_code_stamp = millis();

	// 	prev_got_faultCode = got_faultCode;
	// 	if ((faultCode[0] != 0) || (faultCode[1] != 0)){
	// 		// digitalWrite(LED2_PIN, 1);
	// 		got_faultCode = true;
	// 	} else {
	// 		// digitalWrite(LED2_PIN, 0);
	// 		got_faultCode = false;
	// 	}

	// 	// if ((prev_got_faultCode != got_faultCode) && (got_faultCode == true)){
	// 	// digitalWrite(LED2_PIN, 0);
	// 	// } else if ((prev_got_faultCode != got_faultCode) && (got_faultCode == false)){
	// 	// digitalWrite(LED2_PIN, 1);
	// 	// }

	// }
}