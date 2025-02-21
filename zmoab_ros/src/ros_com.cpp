
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
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8_multi_array.h>

#include "sbus.h"
#include "motor_control.h"
#include "imu_handler.h"
#include "gpio_led.h"
#include "pwm_handler.h"
#include "switch_handler.h"
#include "led_strip.h"

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


/* --------------------------- */
/* --- RCL objects Declare --- */
/* --------------------------- */
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t micros_node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/* -------------------------- */
/* --- ROS Topics Declare --- */
/* -------------------------- */
rcl_publisher_t sbus_pub;
rcl_publisher_t rpm_fb_pub;
rcl_publisher_t cart_mode_pub;
rcl_publisher_t error_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t pwm_in_pub;
rcl_publisher_t switch_pub;
rcl_publisher_t enc_pub; 

rcl_subscription_t rpm_cmd_sub;
rcl_subscription_t cart_mode_cmd_sub;
rcl_subscription_t disable_motor_sub;
rcl_subscription_t servo_sub;
rcl_subscription_t led_strip_sub;
rcl_subscription_t reset_imu_sub;

/* ------------------------ */
/* --- ROS Msg Declare --- */
/* ----------------------- */
std_msgs__msg__UInt16MultiArray sbus_msg;
std_msgs__msg__Int16MultiArray rpm_fb_msg;
std_msgs__msg__Int8 cart_mode_msg;
std_msgs__msg__Int16MultiArray error_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int16MultiArray pwm_in_msg;
std_msgs__msg__Int8MultiArray switch_msg;
std_msgs__msg__Int32MultiArray enc_msg;

std_msgs__msg__Int16MultiArray rpm_cmd_msg;
std_msgs__msg__Int8 cart_mode_cmd_msg;
std_msgs__msg__Bool disable_motor_msg;
std_msgs__msg__Int16MultiArray servo_msg;
std_msgs__msg__Int8 led_strip_msg;
std_msgs__msg__Bool reset_imu_msg;


/* -------------------- */
/* --- ROS Msg init --- */
/* -------------------- */
void init_ros_msgs(){

	static uint16_t _sbus_init[16];
	for (int i=0; i < 16; i++){
		_sbus_init[i] = 1024;
	}
	sbus_msg.data.capacity = 16; // number of array length
	sbus_msg.data.size = 16;     // number of array length
	sbus_msg.data.data = _sbus_init;

	static int16_t _rpm_init[2] = {0,0};
	rpm_fb_msg.data.capacity = 2; // number of array length
	rpm_fb_msg.data.size = 2;     // number of array length
	rpm_fb_msg.data.data = _rpm_init;

	rpm_cmd_msg.data.capacity = 2; // number of array length
	rpm_cmd_msg.data.size = 2;     // number of array length
	rpm_cmd_msg.data.data = _rpm_init;

	static int32_t _encoder_init[2] = {0,0};
	enc_msg.data.capacity = 2; // number of array length
	enc_msg.data.size = 2;     // number of array length
	enc_msg.data.data = _encoder_init;

	cart_mode_msg.data = cart_mode;

	cart_mode_cmd_msg.data = cart_mode;

	disable_motor_msg.data = false;
	
	imu_msg.header.frame_id.data = "imu_link";
	imu_msg.header.stamp.sec = int(millis() / 1000);
	imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());
	imu_msg.orientation.x = qx;
	imu_msg.orientation.y = qy;
	imu_msg.orientation.z = qz;
	imu_msg.orientation.w = qw;
	imu_msg.angular_velocity.x = gx;
	imu_msg.angular_velocity.y = gy;
	imu_msg.angular_velocity.z = gz;
	imu_msg.linear_acceleration.x = ax;
	imu_msg.linear_acceleration.y = ay;
	imu_msg.linear_acceleration.z = az;

	static int16_t _error_init[2] = {0,0};
	error_msg.data.capacity = 2; // number of array length
	error_msg.data.size = 2;     // number of array length
	error_msg.data.data = _error_init;

	static int16_t pwm_data[2] = {1500, 1500};
	pwm_in_msg.data.capacity = 2; // number of array length
	pwm_in_msg.data.size = 2;     // number of array length
	pwm_in_msg.data.data = pwm_data;

	static int16_t servo_data[2] = {1500, 1500};
	servo_msg.data.capacity = 2; // number of array length
	servo_msg.data.size = 2;     // number of array length
	servo_msg.data.data = servo_data;

	static int8_t sw_init[4] = {0,0,0,0};
	switch_msg.data.capacity = 4;
	switch_msg.data.size = 4;
	switch_msg.data.data = sw_init;

	led_strip_msg.data = 0;

	reset_imu_msg.data = false;

}

/* ------------------------ */
/* --- ROS Sub Callback --- */
/* ------------------------ */
void rpm_cmd_callback(const void *msgin) {
	std_msgs__msg__Int16MultiArray * rpm_cmd_msg = (std_msgs__msg__Int16MultiArray *)msgin;
	if (cart_mode == 2){
		int16_t _rpmL = rpm_cmd_msg->data.data[0];
		int16_t _rpmR = rpm_cmd_msg->data.data[1];
		set_rpm_cmd(_rpmL, _rpmR);
		last_recv_rpm_cmd_stamp = millis();
	}
}

void cart_mode_cmd_callback(const void *msgin) {
	std_msgs__msg__Int8 * cart_mode_cmd_msg = (std_msgs__msg__Int8 *)msgin;
	int8_t _cart_mode = cart_mode_cmd_msg->data;
	if ((_cart_mode == 0) || (_cart_mode == 1) || (_cart_mode == 2)){
		set_cart_mode(_cart_mode);
	}

}

void servo_callback(const void *msgin) {
	std_msgs__msg__Int16MultiArray * servo_msg = (std_msgs__msg__Int16MultiArray *)msgin;
	int16_t _pwm1 = servo_msg->data.data[0];
	int16_t _pwm2 = servo_msg->data.data[1];
	write_pwm(_pwm1, _pwm2);
}

void disable_motor_callback(const void *msgin) {
	std_msgs__msg__Bool * disable_motor_msg = (std_msgs__msg__Bool *)msgin;
	if (disable_motor_msg->data){
		set_disable_motor();
	} else {
		set_enable_motor();
	}
}

void led_strip_callback(const void *msgin) {
	std_msgs__msg__Int8 * led_strip_msg = (std_msgs__msg__Int8 *)msgin;
	int8_t _led_strip_state = led_strip_msg->data;
	set_led_strip_state(_led_strip_state);
}

void reset_imu_callback(const void *msgin){
	std_msgs__msg__Bool * reset_imu_msg = (std_msgs__msg__Bool *)msgin;
	if (reset_imu_msg->data == true){
		reset_imu_variables();
		bno055_connected = init_BNO055();
	}
}

/* -------------------------- */
/* --- ROS Timer Callback --- */
/* -------------------------- */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	(void) last_call_time;
	if (timer != NULL) {

		rcl_ret_t ret = RCL_RET_OK;
		
		/// Publish PWM Input ///
		static int16_t _pwm_val[2];
		_pwm_val[0] = get_pwm_val_byID(1);
		_pwm_val[1] = get_pwm_val_byID(2);
		pwm_in_msg.data.data[0] = _pwm_val[0]; //(uint16_t)pwm1_struct.pwm_val;
		pwm_in_msg.data.data[1] = _pwm_val[1]; //(uint16_t)pwm2_struct.pwm_val;
		ret = rcl_publish(&pwm_in_pub, &pwm_in_msg, NULL);

		/// Publish SBUS ///
		memcpy(sbus_msg.data.data, ch, sizeof(ch));
		ret = rcl_publish(&sbus_pub, &sbus_msg, NULL);

		/// Publish IMU data ///
		imu_msg.header.stamp.sec = int(millis() / 1000);
		imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());
		imu_msg.orientation.x = qx;
		imu_msg.orientation.y = qy;
		imu_msg.orientation.z = qz;
		imu_msg.orientation.w = qw;
		imu_msg.angular_velocity.x = gx;
		imu_msg.angular_velocity.y = gy;
		imu_msg.angular_velocity.z = gz;
		imu_msg.linear_acceleration.x = ax;
		imu_msg.linear_acceleration.y = ay;
		imu_msg.linear_acceleration.z = az;
		ret = rcl_publish(&imu_pub, &imu_msg, NULL);

		/// Publish RPM Feedback ///
		static int16_t _rpmFB[2];
		_rpmFB[0] = rpmFB[0];
		_rpmFB[1] = rpmFB[1]; 
		rpm_fb_msg.data.data[0] = _rpmFB[0];
		rpm_fb_msg.data.data[1] = _rpmFB[1];
		ret = rcl_publish(&rpm_fb_pub, &rpm_fb_msg, NULL);

		// Publish Fault Code ///
		static int16_t _error[2];
		_error[0] = faultCode_left;
		_error[1] = faultCode_right;
		error_msg.data.data[0] = _error[0];
		error_msg.data.data[1] = _error[1];
		ret = rcl_publish(&error_pub, &error_msg, NULL);

		/// Publish Encoder count ///
		static int32_t _encoder[2];
		_encoder[0] = encoder_left;
		_encoder[1] = encoder_right;
		enc_msg.data.data[0] = _encoder[0];
		enc_msg.data.data[1] = _encoder[1];
		ret = rcl_publish(&enc_pub, &enc_msg, NULL);

		/// Publish Cart mode ///
		cart_mode_msg.data = cart_mode;
		ret = rcl_publish(&cart_mode_pub, &cart_mode_msg, NULL);

		/// Publish switch states ///
		static int8_t _switch_val[4];
		for (int i=0; i<4; i++){
			_switch_val[i] = sw_val_list[i];
		}
		switch_msg.data.data[0] = _switch_val[0];
		switch_msg.data.data[1] = _switch_val[1];
		switch_msg.data.data[2] = _switch_val[2];
		switch_msg.data.data[3] = _switch_val[3];
		ret = rcl_publish(&switch_pub, &switch_msg, NULL);

	}
}

////////////////////////////////////
/// ROS Init & Destroy Functions ///
////////////////////////////////////
bool create_entities(){
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

	/* ----------------- */
	/* --- Publisher --- */ 
	/* ----------------- */

	rclc_publisher_init_best_effort(
        &sbus_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "/zmoab/sbus_rc_ch");

	rclc_publisher_init_best_effort(
		&rpm_fb_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/rpm_fb");

	rclc_publisher_init_best_effort(
		&enc_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"/zmoab/encoder");

	rclc_publisher_init_best_effort(
		&cart_mode_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		"/zmoab/cart_mode");

	rclc_publisher_init_best_effort(
		&imu_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/zmoab/imu");

	rclc_publisher_init_best_effort(
		&error_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/error");

	rclc_publisher_init_best_effort(
		&pwm_in_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/pwm_in");

	rclc_publisher_init_best_effort(
		&switch_pub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
		"/zmoab/switch");

	/* -------------------- */
	/* --- Subscription --- */ 
	/* -------------------- */

	rclc_subscription_init_best_effort(
		&rpm_cmd_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/rpm_cmd");

	rclc_subscription_init_best_effort(
		&cart_mode_cmd_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		"/zmoab/cart_mode_cmd");

	rclc_subscription_init_best_effort(
		&disable_motor_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"/zmoab/disable_motor");

	rclc_subscription_init_best_effort(
		&servo_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
		"/zmoab/servo_cmd");

	rclc_subscription_init_best_effort(
		&led_strip_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		"/zmoab/led_state");

	rclc_subscription_init_best_effort(
		&reset_imu_sub,
		&micros_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"/zmoab/imu_reset");



	const unsigned int timer_timeout = 50;
	rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

	/*
	* num_handles = total_of_subscriber + timer
	* publisher is not counted
	*/
	unsigned int num_handles = 7;
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, num_handles, &allocator);
	rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &rpm_cmd_sub, &rpm_cmd_msg, &rpm_cmd_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &cart_mode_cmd_sub, &cart_mode_cmd_msg, &cart_mode_cmd_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &disable_motor_sub, &disable_motor_msg, &disable_motor_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &led_strip_sub, &led_strip_msg, &led_strip_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &reset_imu_sub, &reset_imu_msg, &reset_imu_callback, ON_NEW_DATA);
	rclc_executor_add_timer(&executor, &timer);

	return true;
}

void destroy_entities(){
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
	ret = rcl_publisher_fini(&cart_mode_pub, &micros_node);
	ret = rcl_publisher_fini(&error_pub, &micros_node);
	ret = rcl_publisher_fini(&imu_pub, &micros_node);
	ret = rcl_publisher_fini(&pwm_in_pub, &micros_node);
	ret = rcl_publisher_fini(&enc_pub, &micros_node);
	ret = rcl_publisher_fini(&switch_pub, &micros_node);


	ret = rcl_subscription_fini(&rpm_cmd_sub, &micros_node);
	ret = rcl_subscription_fini(&cart_mode_cmd_sub, &micros_node);
	ret = rcl_subscription_fini(&disable_motor_sub, &micros_node);
	ret = rcl_subscription_fini(&servo_sub, &micros_node);
	ret = rcl_subscription_fini(&led_strip_sub, &micros_node);
	ret = rcl_subscription_fini(&reset_imu_sub, &micros_node);

}

void ros_loop(){
	prev_state = state;
	switch (state) {
		case WAITING_AGENT:
			state_connected = false;
			EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				state_connected = false;
				destroy_entities();
			};
			break;
		case AGENT_CONNECTED:
			// if number of MS is too low less than 1000, it will ping agent too many times
			// and caused reconnect too many times
			EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				state_connected = true;
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			}
			break;
		case AGENT_DISCONNECTED:
			destroy_entities();
			state_connected = false;
			set_rpm_cmd(0, 0);
			state = WAITING_AGENT;
			break;
		default:
			break;
	}

}