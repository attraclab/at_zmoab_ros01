#include <Arduino.h>
// #include <ESP32Servo.h> 

// Servo servo1;
// Servo servo2;

int pwm_min = 700;
int pwm_mid = 1500;
int pwm_max = 2300;

#define SERVO1_PIN 5 //7
#define SERVO2_PIN 6 //15

#define PWM1_PIN 7 //5
#define PWM2_PIN 15 //6

struct {
  unsigned long down_stamp;
  unsigned long high_stamp;
  bool allow_cal_pwm = false;

  unsigned long last_log_stamp;
  unsigned long pwm_val;
  bool prev_pin_state = false;
  bool pin_state = false;
} pwm1_struct, pwm2_struct;


bool readPin(int PIN){
	return (GPIO.in1.val >> ( PIN )) & 0x1;
}

void IRAM_ATTR readPWM1_ISR(){

  pwm1_struct.prev_pin_state = pwm1_struct.pin_state;
  pwm1_struct.pin_state = digitalRead(PWM1_PIN);
//   pwm1_struct.pin_state = readPin(PWM1_PIN);
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
//   pwm2_struct.pin_state = readPin(PWM2_PIN);
  if (pwm2_struct.pin_state == true){
	pwm2_struct.high_stamp = micros();
  } else if ((pwm2_struct.prev_pin_state != pwm2_struct.pin_state) && (pwm2_struct.pin_state == false)){
	pwm2_struct.down_stamp = micros();
	pwm2_struct.pwm_val = (pwm2_struct.down_stamp - pwm2_struct.high_stamp);
	pwm2_struct.allow_cal_pwm = true;
  }

}

void writePin(int PIN, bool logic){
	if (logic){
		GPIO.out_w1ts = 1 << PIN;
	} else {
		GPIO.out_w1tc = 1 << PIN;
	}
}


hw_timer_t *servo1_timer = NULL;
hw_timer_t *servo2_timer = NULL;

// int interrupt_time = 5; // microseconds
// int time_period = 20000; // 20ms or 50Hz
// int total_pulse_tick = time_period/interrupt_time; // 20000/5 = 4000tick in one period
// int pulse_count = 0;
// unsigned long pwm_start_on_stamp = micros();
// bool pwm_raise_edge = true;
// uint16_t desired_pwm = 1500;

struct {
	int interrupt_time = 5;
	int time_period = 20000;
	// unsigned long pwm_start_on_stamp = micros();
	uint64_t pwm_start_on_stamp;
	bool pwm_raise_edge = true;
	uint16_t desired_pwm;
} sv1_struc, sv2_struc;

void IRAM_ATTR servo1_pwmGenerate_ISR(){

	if (sv1_struc.pwm_raise_edge){
		writePin(SERVO1_PIN, true);
		// digitalWrite(SERVO1_PIN, HIGH);
		sv1_struc.pwm_start_on_stamp = micros();
		// sv1_struc.pwm_start_on_stamp = timerReadMicros(servo1_timer);
		sv1_struc.pwm_raise_edge = false;
	} else { 
		if ((micros() - sv1_struc.pwm_start_on_stamp) < sv1_struc.desired_pwm){
		// if ((timerReadMicros(servo1_timer) - sv1_struc.pwm_start_on_stamp) < sv1_struc.desired_pwm){

		} else {
			writePin(SERVO1_PIN, false);
			// digitalWrite(SERVO1_PIN, LOW);
			if ((micros() - sv1_struc.pwm_start_on_stamp) >= sv1_struc.time_period){
			// if ((timerReadMicros(servo1_timer) - sv1_struc.pwm_start_on_stamp) >= sv1_struc.time_period){
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
		// sv2_struc.pwm_start_on_stamp = timerReadMicros(servo2_timer);
		sv2_struc.pwm_raise_edge = false;
	} else { 
		if ((micros() - sv2_struc.pwm_start_on_stamp) < sv2_struc.desired_pwm){
		// if ((timerReadMicros(servo2_timer) - sv2_struc.pwm_start_on_stamp) < sv2_struc.desired_pwm){

		} else {
			writePin(SERVO2_PIN, false);
			// digitalWrite(SERVO2_PIN, LOW);
			if ((micros() - sv2_struc.pwm_start_on_stamp) >= sv2_struc.time_period){
			// if ((timerReadMicros(servo2_timer) - sv2_struc.pwm_start_on_stamp) >= sv2_struc.time_period){
				sv2_struc.pwm_raise_edge = true;
			}
		}
	}
}

uint16_t pwm_list[3] = {900, 1500, 2100};
unsigned long last_update_pwm_stamp = millis();
int count = 0;

void setup() {

	USBSerial.begin(115200);

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

  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  // servo1.setPeriodHertz(240);// Standard 50hz servo
  // servo2.setPeriodHertz(240);// Standard 50hz servo

  // servo1.attach(SERVO1_PIN, pwm_min, pwm_max);
  // servo2.attach(SERVO2_PIN, pwm_min, pwm_max);

	pinMode(PWM1_PIN, INPUT);
	attachInterrupt(PWM1_PIN, readPWM1_ISR, CHANGE);

	pwm1_struct.last_log_stamp = millis();

	pinMode(PWM2_PIN, INPUT);
	attachInterrupt(PWM2_PIN, readPWM2_ISR, CHANGE);

	pwm2_struct.last_log_stamp = millis();

}

void loop() {

	// sv1_struc.desired_pwm = 1500;
	// sv2_struc.desired_pwm = 1500;
	// delay(500);
	// sv1_struc.desired_pwm = 900;
	// sv2_struc.desired_pwm = 900;
	// delay(500);
	// sv1_struc.desired_pwm = 2100;
	// sv2_struc.desired_pwm = 2100;
	// delay(500);
	if ((millis() - last_update_pwm_stamp) > 1000){
		sv1_struc.desired_pwm = pwm_list[count];
		sv2_struc.desired_pwm = pwm_list[count];

		count += 1;
		if (count >= 3){
			count = 0;
		}

		last_update_pwm_stamp = millis();
	}


  // servo1.writeMicroseconds(pwm_mid);
  // servo2.writeMicroseconds(pwm_mid);
  // delay(500);

  // servo1.writeMicroseconds(pwm_min);
  // servo2.writeMicroseconds(pwm_min);
  // delay(500);

  // servo1.writeMicroseconds(pwm_max);
  // servo2.writeMicroseconds(pwm_max);
  // delay(500);

  if (pwm1_struct.allow_cal_pwm){

    if ((millis() - pwm1_struct.last_log_stamp) > 10){
      USBSerial.println("PWM1: " + String(pwm1_struct.pwm_val) + "PWM2: " + String(pwm2_struct.pwm_val));
      pwm1_struct.last_log_stamp = millis();
    }
  
  }



}

