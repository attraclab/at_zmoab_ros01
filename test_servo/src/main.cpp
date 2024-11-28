#include <Arduino.h>
#include <ESP32Servo.h> 

Servo servo1;
Servo servo2;

int pwm_min = 700;
int pwm_mid = 1500;
int pwm_max = 2300;

#define SERVO1_PIN 7
#define SERVO2_PIN 15

#define PWM1_PIN 5
#define PWM2_PIN 6

struct {
  unsigned long down_stamp;
  unsigned long high_stamp;
  bool allow_cal_pwm = false;

  unsigned long last_log_stamp;
  unsigned long pwm_val;
  bool prev_pin_state = false;
  bool pin_state = false;
} pwm1_struct, pwm2_struct;



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



void setup() {

  USBSerial.begin(115200);

  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(240);// Standard 50hz servo
  servo2.setPeriodHertz(240);// Standard 50hz servo

  servo1.attach(SERVO1_PIN, pwm_min, pwm_max);
  servo2.attach(SERVO2_PIN, pwm_min, pwm_max);

  pinMode(PWM1_PIN, INPUT);
  attachInterrupt(PWM1_PIN, readPWM1_ISR, CHANGE);

  pwm1_struct.last_log_stamp = millis();

  pinMode(PWM2_PIN, INPUT);
  attachInterrupt(PWM2_PIN, readPWM2_ISR, CHANGE);

  pwm2_struct.last_log_stamp = millis();

}

void loop() {

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

