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
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8_multi_array.h>

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
rcl_publisher_t cart_mode_pub;
// rcl_publisher_t error_left_pub;
// rcl_publisher_t error_right_pub;
rcl_publisher_t error_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t pwm_in_pub;
rcl_publisher_t sw1_pub;
rcl_publisher_t sw2_pub;

rcl_subscription_t rpm_cmd_sub;
rcl_subscription_t servo_sub;
rcl_subscription_t cart_mode_cmd_sub;
rcl_subscription_t disable_motor_sub;
rcl_subscription_t led_strip_sub;
rcl_subscription_t reset_imu_sub;


std_msgs__msg__UInt16MultiArray sbus_msg;
std_msgs__msg__Int16MultiArray rpm_fb_msg;
std_msgs__msg__Int8 cart_mode_msg;
// std_msgs__msg__Int16 error_left_msg;
// std_msgs__msg__Int16 error_right_msg;
std_msgs__msg__Int16MultiArray error_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int16MultiArray pwm_in_msg;
std_msgs__msg__Bool sw1_msg;
std_msgs__msg__Bool sw2_msg;

std_msgs__msg__Int16MultiArray rpm_cmd_msg;
std_msgs__msg__Int16MultiArray servo_msg;
std_msgs__msg__Int8 cart_mode_cmd_msg;
std_msgs__msg__Bool disable_motor_msg;
std_msgs__msg__Int8 led_strip_msg;
std_msgs__msg__Bool reset_imu_msg;

double qw = 1.0; 
double qx = 0.0;
double qy = 0.0;
double qz = 0.0;
double ax = 0.0;
double ay = 0.0;
double az = 0.0;
double gx = 0.0;
double gy = 0.0;
double gz = 0.0;

void rpm_cmd_callback(const void *msgin) {
	std_msgs__msg__Int16MultiArray * rpm_cmd_msg = (std_msgs__msg__Int16MultiArray *)msgin;
	// if (cart_mode == 2){
	// 	int16_t _rpmL = rpm_cmd_msg->data.data[0];
	// 	int16_t _rpmR = rpm_cmd_msg->data.data[1];
	// 	set_rpm_cmd(_rpmL, _rpmR);
	// 	last_recv_rpm_cmd_stamp = millis();
	// }
}

void cart_mode_cmd_callback(const void *msgin) {
	std_msgs__msg__Int8 * cart_mode_cmd_msg = (std_msgs__msg__Int8 *)msgin;
	// int8_t _cart_mode = cart_mode_cmd_msg->data;
	// if ((_cart_mode == 0) || (_cart_mode == 1) || (_cart_mode == 2)){
	// 	set_cart_mode(_cart_mode);
	// }
	
}

void servo_callback(const void *msgin) {
	std_msgs__msg__Int16MultiArray * servo_msg = (std_msgs__msg__Int16MultiArray *)msgin;
	// int16_t _pwm1 = servo_msg->data.data[0];
	// int16_t _pwm2 = servo_msg->data.data[1];
	// write_pwm(_pwm1, _pwm2);
}

void disable_motor_callback(const void *msgin) {
	std_msgs__msg__Bool * disable_motor_msg = (std_msgs__msg__Bool *)msgin;  
	// if (disable_motor_msg->data){
	// 	set_disable_motor();
	// } else {
	// 	set_enable_motor();
	// }
}

void led_strip_callback(const void *msgin) {
	std_msgs__msg__Int8 * led_strip_msg = (std_msgs__msg__Int8 *)msgin;
	// int8_t _led_strip_state = led_strip_msg->data;
	// set_led_strip_state(_led_strip_state);
	
}

void reset_imu_callback(const void *msgin){
	std_msgs__msg__Bool * reset_imu_msg = (std_msgs__msg__Bool *)msgin;
	// if (reset_imu_msg->data == true){
	// 	reset_imu_variables();
	// 	bno055_connected = init_BNO055();
	// }
}

static int16_t _pwm_val[2] = {1500,1500};

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	(void) last_call_time;
	if (timer != NULL) {

		rcl_ret_t ret = RCL_RET_OK;
		
		/// Publish PWM Input ///
		
        _pwm_val[0] += 1;
        _pwm_val[1] += 1;
		// _pwm_val[0] = get_pwm_val_byID(1);
		// _pwm_val[1] = get_pwm_val_byID(2);
		// get_pwm_val(_pwm_val);
		// pwm_in_msg.data.data[0] = _pwm_val[0]; //(uint16_t)pwm1_struct.pwm_val;
		// pwm_in_msg.data.data[1] = _pwm_val[1]; //(uint16_t)pwm2_struct.pwm_val;
		// pwm_in_msg.data.data[0] = get_pwm_val_byID(1); //(uint16_t)pwm1_struct.pwm_val;
		// pwm_in_msg.data.data[1] = get_pwm_val_byID(2); //(uint16_t)pwm2_struct.pwm_val;
		// memcpy(_pwm_val, pwm_in_msg.data.data, sizeof(pwm_in_msg.data.data));
        pwm_in_msg.data.data[0] = _pwm_val[0];
        pwm_in_msg.data.data[1] = _pwm_val[1];
		ret = rcl_publish(&pwm_in_pub, &pwm_in_msg, NULL);

		/// Publish SBUS ///
		// memcpy(sbus_msg.data.data, ch, sizeof(ch));
		ret = rcl_publish(&sbus_pub, &sbus_msg, NULL);

		/// Publish IMU data ///
		imu_msg.header.stamp.sec = int(millis() / 1000);
		imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());
		imu_msg.orientation.x += 0.01;
		imu_msg.orientation.y += 0.01;
		imu_msg.orientation.z += 0.01;
		imu_msg.orientation.w += 0.01;
		imu_msg.angular_velocity.x += 0.01;
		imu_msg.angular_velocity.y += 0.01;
		imu_msg.angular_velocity.z += 0.01;
		imu_msg.linear_acceleration.x += 0.01;
		imu_msg.linear_acceleration.y += 0.01;
		imu_msg.linear_acceleration.z += 0.01;
		ret = rcl_publish(&imu_pub, &imu_msg, NULL);

		/// Publish RPM Feedback ///
		rpm_fb_msg.data.data[0] += 3;
		rpm_fb_msg.data.data[1] += 2;
		ret = rcl_publish(&rpm_fb_pub, &rpm_fb_msg, NULL);

		// Publish Fault Code ///
		// fault_code_msg.data.data[0] = faultCode[0];
		// fault_code_msg.data.data[1] = faultCode[1];
		// ret = rcl_publish(&fault_code_pub, &fault_code_msg, NULL);
		// error_left_msg.data = faultCode_left;
		// error_right_msg.data = faultCode_right;
		// ret = rcl_publish(&error_left_pub, &error_left_msg, NULL);
		// ret = rcl_publish(&error_right_pub, &error_right_msg, NULL);
        ret = rcl_publish(&error_pub, &error_msg, NULL);

		/// Publish Encoder count ///
		// enc_msg.data.data[0] = encoder_left;
		// enc_msg.data.data[1] = encoder_right;
		// ret = rcl_publish(&enc_pub, &enc_msg, NULL);

		// enc_left_msg.data = encoder_left;
		// enc_right_msg.data = encoder_right;
		// ret = rcl_publish(&enc_left_pub, &enc_left_msg, NULL);
		// ret = rcl_publish(&enc_right_pub, &enc_right_msg, NULL);

		/// Publish Cart mode ///
		// cart_mode_msg.data = cart_mode;
		ret = rcl_publish(&cart_mode_pub, &cart_mode_msg, NULL);

		// int8_t sw_val_list[4];
		// for (int i; i<4; i++){
		// 	bool sw_val = get_switch_value(i+1);
		// 	if (sw_val){
		// 		sw_val_list[i] = 1;
		// 	} else {
		// 		sw_val_list[i] = 0;
		// 	}
		// }
		// memcpy(switch_msg.data.data, sw_val_list, sizeof(sw_val_list));
		// ret = rcl_publish(&switch_pub, &switch_msg, NULL);
		// sw1_msg.data = sw_val_list[0];
		ret = rcl_publish(&sw1_pub, &sw1_msg, NULL);

		// sw2_msg.data = sw_val_list[1];
		ret = rcl_publish(&sw2_pub, &sw2_msg, NULL);

	}
}

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

	// rclc_publisher_init(
	// 	&sbus_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
	// 	"/zmoab/sbus_rc_ch", &rmw_qos_profile_default);
    rclc_publisher_init_best_effort(
        &sbus_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "/zmoab/sbus_rc_ch");

	// rclc_publisher_init(
	// 	&rpm_fb_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/rpm_fb", &rmw_qos_profile_default);
    rclc_publisher_init_best_effort(
        &rpm_fb_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/zmoab/rpm_fb");

	// rclc_publisher_init(
	// 	&enc_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	// 	"/zmoab/enc", &rmw_qos_profile_default);

	// rclc_publisher_init(
	// 	&enc_left_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"/zmoab/enc_left", &rmw_qos_profile_default);

	// rclc_publisher_init(
	// 	&enc_right_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"/zmoab/enc_right", &rmw_qos_profile_default);

	// rclc_publisher_init(
	// 	&cart_mode_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
	// 	"/zmoab/cart_mode", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &cart_mode_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/zmoab/cart_mode");

	// rclc_publisher_init(
	// 	&imu_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	// 	"/zmoab/imu", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &imu_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/zmoab/imu");

	// rclc_publisher_init(
	// 	&error_pub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/error", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &error_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/zmoab/error");

    // rclc_publisher_init(
    //     &pwm_in_pub,
    //     &micros_node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    //     "/zmoab/pwm_in", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &pwm_in_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/zmoab/pwm_in");

    // rclc_publisher_init(
    //     &sw1_pub,
    //     &micros_node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //     "/zmoab/sw1", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &sw1_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/zmoab/sw1");

    // rclc_publisher_init(
    //     &sw2_pub,
    //     &micros_node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //     "/zmoab/sw2", &rmw_qos_profile_default);

    rclc_publisher_init_best_effort(
        &sw2_pub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/zmoab/sw2");




    // rclc_subscription_init(
	// 	&rpm_cmd_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/rpm_cmd", &rmw_qos_profile_default);

    rclc_subscription_init_best_effort(
        &rpm_cmd_sub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/zmoab/rpm_cmd");

	// rclc_subscription_init(
	// 	&cart_mode_cmd_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
	// 	"/zmoab/cart_mode_cmd", &rmw_qos_profile_default);

    rclc_subscription_init_best_effort(
        &cart_mode_cmd_sub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/zmoab/cart_mode_cmd");

	// rclc_subscription_init(
	// 	&disable_motor_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
	// 	"/zmoab/disable_motor", &rmw_qos_profile_default);

    rclc_subscription_init_best_effort(
        &disable_motor_sub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/zmoab/disable_motor");

	// rclc_subscription_init(
	// 	&servo_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
	// 	"/zmoab/servo_cmd", &rmw_qos_profile_default);

    rclc_subscription_init_best_effort(
        &servo_sub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "/zmoab/servo_cmd");

	// rclc_subscription_init(
	// 	&led_strip_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
	// 	"/zmoab/led_state", &rmw_qos_profile_default);

    rclc_subscription_init_best_effort(
        &led_strip_sub,
        &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/zmoab/led_state");

	// rclc_subscription_init(
	// 	&reset_imu_sub,
	// 	&micros_node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
	// 	"/zmoab/imu_reset", &rmw_qos_profile_default);

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
	// rclc_executor_add_subscription(&executor, &pack_sub, &pack_msg, &pack_callback, ON_NEW_DATA);
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
	ret = rcl_subscription_fini(&rpm_cmd_sub, &micros_node);
	ret = rcl_publisher_fini(&error_pub, &micros_node);
	// ret = rcl_publisher_fini(&error_left_pub, &micros_node);
	// ret = rcl_publisher_fini(&error_right_pub, &micros_node);
	ret = rcl_subscription_fini(&cart_mode_cmd_sub, &micros_node);
	ret = rcl_subscription_fini(&disable_motor_sub, &micros_node);
	ret = rcl_publisher_fini(&imu_pub, &micros_node);
	ret = rcl_subscription_fini(&servo_sub, &micros_node);
	ret = rcl_publisher_fini(&pwm_in_pub, &micros_node);
	// ret = rcl_publisher_fini(&enc_pub, &micros_node);
	// ret = rcl_publisher_fini(&enc_left_pub, &micros_node);
	// ret = rcl_publisher_fini(&enc_right_pub, &micros_node);

	ret = rcl_publisher_fini(&sw1_pub, &micros_node);
	ret = rcl_publisher_fini(&sw2_pub, &micros_node);
	ret = rcl_subscription_fini(&led_strip_sub, &micros_node);
	ret = rcl_subscription_fini(&reset_imu_sub, &micros_node);

}


void setup() {

    USBSerial.begin(115200);
    set_microros_serial_transports(USBSerial);
    delay(2000);

    state = WAITING_AGENT;

    /* ------------ */
	/* ROS Msg init */
	/* ------------ */
    static uint16_t sbus_ch[16];
    for (int i=0; i<16; i++){
        sbus_ch[i] = 1024;
    }
	sbus_msg.data.capacity = 16; // number of array length
	sbus_msg.data.size = 16;     // number of array length
	sbus_msg.data.data = sbus_ch;
    // memcpy(sbus_ch, sbus_msg.data.data, sizeof(sbus_msg.data.data));

    static int16_t zero_rpm[2] = {0,0};
	rpm_fb_msg.data.capacity = 2; // number of array length
	rpm_fb_msg.data.size = 2;     // number of array length
	rpm_fb_msg.data.data = zero_rpm;

	static int16_t zero_data[2] = {0, 0};
	rpm_cmd_msg.data.capacity = 2; // number of array length
	rpm_cmd_msg.data.size = 2;     // number of array length
	rpm_cmd_msg.data.data = zero_data;

	// int32_t _encoder_init[2] = {0,0};
	// enc_msg.data.capacity = 2; // number of array length
	// enc_msg.data.size = 2;     // number of array length
	// enc_msg.data.data = _encoder_init;

	// enc_left_msg.data = encoder_left;
	// enc_right_msg.data = encoder_right;

	cart_mode_msg.data = 0;

	cart_mode_cmd_msg.data = 0;

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
	// error_left_msg.data = 0;
	// error_right_msg.data = 0;

	static int16_t pwm_data[2] = {1500, 1500};
	pwm_in_msg.data.capacity = 2; // number of array length
	pwm_in_msg.data.size = 2;     // number of array length
	pwm_in_msg.data.data = pwm_data;

	static int16_t servo_data[2] = {1500, 1500};
	servo_msg.data.capacity = 2; // number of array length
	servo_msg.data.size = 2;     // number of array length
	servo_msg.data.data = servo_data;

	// int8_t sw_init[4] = {0,0,0,0};
	// switch_msg.data.capacity = 4;
	// switch_msg.data.size = 4;
	// switch_msg.data.data = sw_init;
	sw1_msg.data = false;
	sw2_msg.data = false;

	led_strip_msg.data = 0;
	reset_imu_msg.data = false;


}

void loop() {

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

}

