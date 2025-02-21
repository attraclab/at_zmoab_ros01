#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <Arduino.h>
#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include "sbus.h"


extern int16_t rpmFB[2];
extern int32_t encoderFB[2];
extern int16_t faultCode[2];
extern unsigned long last_recv_rpm_cmd_stamp;


void channelMixing(uint16_t str_ch, uint16_t thr_ch, float _rpm[2]);
void setup_motor();
void set_enable_motor(uint8_t flag);
void set_rpm_cmd(int16_t _rpmL, int16_t _rpmR);

void motor_control_loop();

#endif