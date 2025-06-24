#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <Arduino.h>
#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include "sbus.h"


extern int16_t rpmFB[2];
extern int32_t encoder_left;
extern int32_t encoder_right;
extern int16_t faultCode_left;
extern int16_t faultCode_right;
extern unsigned long last_recv_rpm_cmd_stamp;


void channelMixing(uint16_t str_ch, uint16_t thr_ch, float _rpm[2]);
void channelMixing_by_Joystick(float x, float y, float _rpm[2]);
void setup_motor();
void set_enable_motor();
void set_disable_motor();
void set_rpm_cmd(int16_t _rpmL, int16_t _rpmR);
void init_drive();
// static void motor_worker_task(void *pvParameters);

void motor_control_loop();

#endif