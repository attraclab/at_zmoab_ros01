#ifndef __PWM_H__
#define __PWM_H__

#include <Arduino.h>

void setup_servo();
void IRAM_ATTR readPWM1_ISR();
void IRAM_ATTR readPWM2_ISR();
int16_t get_pwm_val_byID(int _id);
void IRAM_ATTR servo1_pwmGenerate_ISR();
void IRAM_ATTR servo2_pwmGenerate_ISR();
void write_pwm(int16_t pwm1, int16_t pwm2);
void writePin(int PIN, bool logic);


#endif