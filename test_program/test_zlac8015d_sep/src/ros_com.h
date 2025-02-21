#ifndef __ROS_COM_H__
#define __ROS_COM_H__

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int8.h>



extern bool state_connected;

void init_ros_msgs();
bool create_entities();
void destroy_entities();
void ros_loop();

#endif