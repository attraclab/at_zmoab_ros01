
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

rcl_publisher_t int_pub;
std_msgs__msg__Int8 int_msg;




void init_ros_msgs(){

    int_msg.data = 0;

}



void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    rcl_ret_t ret = RCL_RET_OK;

    /// Publish Cart mode ///
    int_msg.data += 1;
    ret = rcl_publish(&int_pub, &int_msg, NULL);

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
    &int_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/int_pub", &rmw_qos_profile_default);

  
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   */
  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

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

  ret = rcl_publisher_fini(&int_pub, &micros_node);
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
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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