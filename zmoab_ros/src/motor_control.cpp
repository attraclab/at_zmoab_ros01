
#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include "motor_control.h"
#include "sbus.h"
#include "gpio_led.h"

/* ------------------------ */
/* ZLAC8015D & ModbusMaster */
/* ------------------------ */
#define MAX485_DE  2
#define MODBUS_RX_PIN 43 //44
#define MODBUS_TX_PIN 44 //43

ModbusMaster ModbusNode;
ZLAC8015D driver;

uint8_t res;

int16_t rpmL = 0;
int16_t rpmR = 0;
float rpm[2];
float prev_y = 0.0;
float max_rpm = 200.0;
float rpmDB = 5.0;
float ble_max_rpm = 100.0;


unsigned long last_fault_code_stamp;
bool got_faultCode = false;
bool prev_got_faultCode = false;

int16_t _rpmFB[2];
int32_t _encoderFB[2];
int16_t _faultCode[2];

int16_t rpmFB[2] = {0, 0};
int32_t encoder_left = 0;
int32_t encoder_right = 0;
int16_t faultCode_left =  0;
int16_t faultCode_right = 0;
unsigned long last_recv_rpm_cmd_stamp = millis();
unsigned long last_loop_stamp = millis();
bool first_drive = false;


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

void channelMixing_by_Joystick(float _x, float _y, float _rpm[2]){

	float x = _x*100.0;
	float y = _y*100.0;

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

	_rpm[0] = map(left, -200.0, 200.0, -ble_max_rpm, ble_max_rpm);
	_rpm[1] = map(right, -200.0, 200.0, -ble_max_rpm, ble_max_rpm);

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

	do {
		res = driver.disable_motor();
	} while (res != 0);
	delay(100);

	res = driver.set_mode(3);
	res = driver.enable_motor();
	res = driver.set_accel_time(20, 20);
	res = driver.set_decel_time(20, 20);

	delay(1000);
	last_fault_code_stamp = millis();

	/// Trying separate thread but disconnected with agent many times
	// xTaskCreate(motor_worker_task, "motor_worker_task", 2048, NULL, 12, NULL);
}

void set_enable_motor(){
	res = driver.enable_motor();
}

void set_disable_motor(){
	res = driver.disable_motor();
}

void set_rpm_cmd(int16_t _rpmL, int16_t _rpmR){
	rpmL = _rpmL;
	rpmR = _rpmR;
}

void init_drive(){

	/* There is somebug on ModbusMaster when we firstly send rpm command
	which not 0 value. It will block the code around 2 seconds, and the
	cart just keep moving with that speed...
	So I make it moves 2rpm rotating then stop, so next rpm command will not
	be blocked anymore */

	do {
		res = driver.set_rpm(2, -2);
		// USBSerial.printf("Try  init rpm\n");
		delay(100);
	} while (res != 0);
	delay(1000);
		
	do {
		res = driver.set_rpm(0, 0);
		// USBSerial.printf("Finish init rpm\n");
		delay(100);
	} while (res != 0);
	delay(1000);

	do {
		res = driver.set_rpm(-2, 2);
		// USBSerial.printf("Move back rpm\n");
		delay(100);
	} while (res != 0);
	delay(1000);

	do {
		res = driver.set_rpm(0, 0);
		// USBSerial.printf("Finish init rpm\n");
		delay(100);
	} while (res != 0);

}

void motor_control_loop(){

	// with everything, loop takes 15ms
	// without read encoder, loop takes 9-10ms
	// withou read encoder/rpm, loop takes 5ms

	unsigned long start_stamp = millis();

	if (first_drive == false){
		first_drive = true;
		init_drive();
		delay(2000);
	}

	if (cart_mode == 2){
		if ((millis() - last_recv_rpm_cmd_stamp) >= 500){
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

	encoder_left = _encoderFB[0];
	encoder_right = _encoderFB[1];

	/* -------------------- */
	/* Check ESC Fault Code */
	/* -------------------- */

	if ((millis() - last_fault_code_stamp) >= 2000){
		res = driver.get_fault_code(_faultCode);

		faultCode_left = _faultCode[0];
		faultCode_right = _faultCode[1];
		
		last_fault_code_stamp = millis();

		prev_got_faultCode = got_faultCode;
		if ((faultCode_left != 0) || (faultCode_right != 0)){
			got_faultCode = true;
		} else {
			got_faultCode = false;
		}

		if ((prev_got_faultCode != got_faultCode) && (got_faultCode == true)){
			set_boardLED_byId(2, true);
		} else if ((prev_got_faultCode != got_faultCode) && (got_faultCode == false)){
			set_boardLED_byId(2, false);
		}

	}

	// unsigned long loop_period = millis() - start_stamp;
	// USBSerial.printf("period: %d\n", loop_period);

	last_loop_stamp = millis();
}

// static void motor_worker_task(void *pvParameters){
// 	while (1){
// 		unsigned long loop_period =  (millis() - last_loop_stamp);
// 		if (loop_period >= 100){
// 			// USBSerial.printf("----------------------\n");
// 			USBSerial.printf("loop_period: %d\n", loop_period);		
// 		}
// 	}
// }