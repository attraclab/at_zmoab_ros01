
#include "ros_com.h"

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

#include "sbus.h"
#include "motor_control.h"


#define EXECUTE_EVERY_N_MS(MS, X)  do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state, prev_state;

bool state_connected = false;



rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t micros_node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t sbus_pub;
rcl_publisher_t rpm_fb_pub;
rcl_subscription_t rpm_cmd_sub;
// rcl_publisher_t encoder_pub;
// rcl_publisher_t fault_code_pub;
rcl_publisher_t cart_mode_pub;
rcl_publisher_t rpm_rep_pub;


std_msgs__msg__UInt16MultiArray sbus_msg;
std_msgs__msg__Int16MultiArray rpm_fb_msg;
std_msgs__msg__Int16MultiArray rpm_cmd_msg;
// std_msgs__msg__Int32MultiArray encoder_msg;
// std_msgs__msg__Int16MultiArray fault_code_msg;
std_msgs__msg__Int8 cart_mode_msg;
std_msgs__msg__Int16MultiArray rpm_rep_msg;


void init_ros_msgs(){

	sbus_msg.data.capacity = 16; // number of array length
	sbus_msg.data.size = 16;     // number of array length
	sbus_msg.data.data = sbus_ch;

	rpm_fb_msg.data.capacity = 2;
	rpm_fb_msg.data.size = 2;
	rpm_fb_msg.data.data = rpmFB;

	int16_t zero_data[2] = {0, 0};
	rpm_cmd_msg.data.capacity = 2; // number of array length
	rpm_cmd_msg.data.size = 2;     // number of array length
	rpm_cmd_msg.data.data = zero_data;

	// int16_t _fault_code_init[2] = {0,0};
	// fault_code_msg.data.capacity = 2; // number of array length
	// fault_code_msg.data.size = 2;     // number of array length
	// fault_code_msg.data.data = _fault_code_init;

	// int32_t _encoder_init[2] = {0,0};
	// encoder_msg.data.capacity = 2; // number of array length
	// encoder_msg.data.size = 2;     // number of array length
	// encoder_msg.data.data = _encoder_init;

	cart_mode_msg.data = cart_mode;

	rpm_rep_msg.data.capacity = 2; // number of array length
	rpm_rep_msg.data.size = 2;     // number of array length
	rpm_rep_msg.data.data = zero_data;

}

void rpm_cmd_callback(const void *msgin) {

	std_msgs__msg__Int16MultiArray * rpm_cmd_msg = (std_msgs__msg__Int16MultiArray *)msgin;

	if (cart_mode == 2){
		int16_t _rpmL = rpm_cmd_msg->data.data[0];
		int16_t _rpmR = rpm_cmd_msg->data.data[1];
		set_rpm_cmd(_rpmL, _rpmR);

		last_recv_rpm_cmd_stamp = millis();
	}

	// rcl_ret_t ret = RCL_RET_OK;
	// rpm_rep_msg.data.data[0] = rpm_cmd_msg->data.data[0];
	// rpm_rep_msg.data.data[1] = rpm_cmd_msg->data.data[1];
	// ret = rcl_publish(&rpm_rep_pub, &rpm_rep_msg, NULL);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

	rcl_ret_t ret = RCL_RET_OK;

	/// Publish SBUS ///
	memcpy(sbus_msg.data.data, ch, sizeof(ch));
	ret = rcl_publish(&sbus_pub, &sbus_msg, NULL);

	rpm_fb_msg.data.data[0] = rpmFB[0];
	rpm_fb_msg.data.data[1] = rpmFB[1];
	ret = rcl_publish(&rpm_fb_pub, &rpm_fb_msg, NULL);

	// /// Publish Fault Code ///
	// fault_code_msg.data.data[0] = 2; //faultCode[0];
	// fault_code_msg.data.data[1] = 3; //faultCode[1];
	// ret = rcl_publish(&fault_code_pub, &fault_code_msg, NULL);

	/// Publish Encoder count ///
	// encoder_msg.data.data[0] = encoderFB[0];
	// encoder_msg.data.data[1] = encoderFB[1];
	// ret = rcl_publish(&encoder_pub, &encoder_msg, NULL);

	/// Publish Cart mode ///
	cart_mode_msg.data = cart_mode;
	ret = rcl_publish(&cart_mode_pub, &cart_mode_msg, NULL);

  }
}

////////////////////////////////////
/// ROS Init & Destroy Functions ///
////////////////////////////////////
bool create_entities()
{
	/*
		TODO : Define your
		- ROS node name
		- namespace
		- ROS_DOMAIN_ID
	*/
	const char * node_name = "at_zmoab_ros";
	const char * ns = "";
	const int domain_id = 0;
  
	/*
	* Initialize node
	*/
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_t ret = RCL_RET_OK;
	ret = rcl_init_options_init(&init_options, allocator);
	ret = rcl_init_options_set_domain_id(&init_options, domain_id);
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
	rclc_node_init_default(&micros_node, node_name, ns, &support);

	rclc_publisher_init(
		&sbus_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
		"/zmoab/sbus_rc_ch", &rmw_qos_profile_default);
	// rclc_publisher_init_best_effort(
	// 	&sbus_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
	// 	"/zmoab/sbus_rc_ch");

	rclc_publisher_init(
		&rpm_fb_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/rpm_fb", &rmw_qos_profile_default);
	// rclc_publisher_init_best_effort(
	// 	&rpm_fb_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/rpm_fb");

	// rclc_publisher_init(
	// 	&fault_code_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/fault_code", &rmw_qos_profile_default);

	// rclc_publisher_init(
	// 	&encoder_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	// 	"/zmoab/encoder", &rmw_qos_profile_default);
	// rclc_publisher_init_best_effort(
	// 	&encoder_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	// 	"/zmoab/encoder");

	rclc_subscription_init(
		&rpm_cmd_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/rpm_cmd", &rmw_qos_profile_default);
	// rclc_subscription_init_best_effort(
	// 	&rpm_cmd_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/rpm_cmd");

	rclc_publisher_init(
		&cart_mode_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		"/zmoab/cart_mode", &rmw_qos_profile_default);
	// rclc_publisher_init_best_effort(
	// 	&cart_mode_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
	// 	"/zmoab/cart_mode");

	rclc_publisher_init(
		&rpm_rep_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/rpm_rep", &rmw_qos_profile_default);

  
	const unsigned int timer_timeout = 50;
	rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

	/*
	* num_handles = total_of_subscriber + timer
	* publisher is not counted
	*/
	unsigned int num_handles = 2;
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, num_handles, &allocator);
	rclc_executor_add_subscription(&executor, &rpm_cmd_sub, &rpm_cmd_msg, &rpm_cmd_callback, ON_NEW_DATA);
	rclc_executor_add_timer(&executor, &timer);

	return true;
}

void destroy_entities()
{
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	rcl_ret_t ret = RCL_RET_OK;

	ret = rcl_timer_fini(&timer);
	ret = rclc_executor_fini(&executor);
	ret = rcl_init_options_fini(&init_options);
	ret = rcl_node_fini(&micros_node);
	rclc_support_fini(&support);

	ret = rcl_publisher_fini(&sbus_pub, &micros_node);
	ret = rcl_publisher_fini(&rpm_fb_pub, &micros_node);
	// ret = rcl_publisher_fini(&encoder_pub, &micros_node);
	ret = rcl_subscription_fini(&rpm_cmd_sub, &micros_node);
	ret = rcl_publisher_fini(&cart_mode_pub, &micros_node);
	ret = rcl_publisher_fini(&rpm_rep_pub, &micros_node);
}

void ros_loop(){
	prev_state = state;
	switch (state) {
		case WAITING_AGENT:
			state_connected = false;
			EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				state_connected = false;
				destroy_entities();
			};
			break;
		case AGENT_CONNECTED:
			EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				state_connected = true;
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			}
			break;
		case AGENT_DISCONNECTED:
			destroy_entities();
			state_connected = false;
			state = WAITING_AGENT;
			break;
		default:
			break;
	}

	// rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}