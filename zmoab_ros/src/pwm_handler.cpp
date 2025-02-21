#include "pwm_handler.h"
 
#define SERVO1_PIN 5 //7
#define SERVO2_PIN 6 //15

#define PWM1_PIN 7 //5
#define PWM2_PIN 15 //6

int pwm_min = 700;
int pwm_mid = 1500;
int pwm_max = 2300;

hw_timer_t *servo1_timer = NULL;
hw_timer_t *servo2_timer = NULL;

/// PWM IN Struct ///
struct {
	unsigned long down_stamp;
	unsigned long high_stamp;
	bool allow_cal_pwm = false;

	unsigned long last_log_stamp;
	unsigned long pwm_val;
	bool prev_pin_state = false;
	bool pin_state = false;
} pwm1_struct, pwm2_struct;

/// SERVO Struct ///
struct {
	int time_period = 20000;
	// unsigned long pwm_start_on_stamp = micros();
	uint64_t pwm_start_on_stamp;
	bool pwm_raise_edge = true;
	int16_t desired_pwm;
} sv1_struc, sv2_struc;


void setup_servo(){
	
	/* ------------------- */
	/* PWM_OUT Servo setup */
	/* ------------------- */
	pinMode(SERVO1_PIN, OUTPUT);
	pinMode(SERVO2_PIN, OUTPUT);

	servo1_timer = timerBegin(1, 80, true);
	timerAttachInterrupt(servo1_timer, &servo1_pwmGenerate_ISR, true);
	timerAlarmWrite(servo1_timer, 20, true); //microseconds
	timerAlarmEnable(servo1_timer);

	servo2_timer = timerBegin(2, 80, true);
	timerAttachInterrupt(servo2_timer, &servo2_pwmGenerate_ISR, true);
	timerAlarmWrite(servo2_timer, 20, true); //microseconds
	timerAlarmEnable(servo2_timer);

	/* ---------------- */
	/* PWM_IN PWM setup */
	/* ---------------- */
	pinMode(PWM1_PIN, INPUT);
	attachInterrupt(PWM1_PIN, readPWM1_ISR, CHANGE);
	pwm1_struct.last_log_stamp = millis();
	pinMode(PWM2_PIN, INPUT);
	attachInterrupt(PWM2_PIN, readPWM2_ISR, CHANGE);
	pwm2_struct.last_log_stamp = millis();
}

void IRAM_ATTR readPWM1_ISR(){
	pwm1_struct.prev_pin_state = pwm1_struct.pin_state;
	pwm1_struct.pin_state = digitalRead(PWM1_PIN);
	if (pwm1_struct.pin_state == true){
		pwm1_struct.high_stamp = micros();
	} else if ((pwm1_struct.prev_pin_state != pwm1_struct.pin_state) && (pwm1_struct.pin_state == false)){
		pwm1_struct.down_stamp = micros();
		pwm1_struct.pwm_val = (pwm1_struct.down_stamp - pwm1_struct.high_stamp);
		pwm1_struct.allow_cal_pwm = true;
	}
}

void IRAM_ATTR readPWM2_ISR(){
	pwm2_struct.prev_pin_state = pwm2_struct.pin_state;
	pwm2_struct.pin_state = digitalRead(PWM2_PIN);
	if (pwm2_struct.pin_state == true){
		pwm2_struct.high_stamp = micros();
	} else if ((pwm2_struct.prev_pin_state != pwm2_struct.pin_state) && (pwm2_struct.pin_state == false)){
		pwm2_struct.down_stamp = micros();
		pwm2_struct.pwm_val = (pwm2_struct.down_stamp - pwm2_struct.high_stamp);
		pwm2_struct.allow_cal_pwm = true;
	}
}

void IRAM_ATTR servo1_pwmGenerate_ISR(){

	if (sv1_struc.pwm_raise_edge){
		writePin(SERVO1_PIN, true);
		// digitalWrite(SERVO1_PIN, HIGH);
		sv1_struc.pwm_start_on_stamp = micros();
		sv1_struc.pwm_raise_edge = false;
	} else { 
		if ((micros() - sv1_struc.pwm_start_on_stamp) < sv1_struc.desired_pwm){
		} else {
			writePin(SERVO1_PIN, false);
			// digitalWrite(SERVO1_PIN, LOW);
			if ((micros() - sv1_struc.pwm_start_on_stamp) >= sv1_struc.time_period){
				sv1_struc.pwm_raise_edge = true;
			}
		}
	}
}

void IRAM_ATTR servo2_pwmGenerate_ISR(){

	if (sv2_struc.pwm_raise_edge){
		writePin(SERVO2_PIN, true);
		// digitalWrite(SERVO2_PIN, HIGH);
		sv2_struc.pwm_start_on_stamp = micros();
		sv2_struc.pwm_raise_edge = false;
	} else { 
		if ((micros() - sv2_struc.pwm_start_on_stamp) < sv2_struc.desired_pwm){
		} else {
			writePin(SERVO2_PIN, false);
			// digitalWrite(SERVO2_PIN, LOW);
			if ((micros() - sv2_struc.pwm_start_on_stamp) >= sv2_struc.time_period){
				sv2_struc.pwm_raise_edge = true;
			}
		}
	}
}

int16_t get_pwm_val_byID(int _id){
	if (_id == 1){
		return (int16_t)pwm1_struct.pwm_val;
	} else if (_id == 2){
		return (int16_t)pwm2_struct.pwm_val;
	} else {
		return 1500;
	}
}

void writePin(int PIN, bool logic){
	if (logic){
		GPIO.out_w1ts = 1 << PIN;
	} else {
		GPIO.out_w1tc = 1 << PIN;
	}
}

void write_pwm(int16_t pwm1, int16_t pwm2){
	sv1_struc.desired_pwm = pwm1;
	sv2_struc.desired_pwm = pwm2;
}