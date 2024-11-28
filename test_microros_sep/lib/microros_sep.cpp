#include "microros_sep.h"

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
void led_callback(const void *msgin) {

  std_msgs__msg__Bool * led_msg = (std_msgs__msg__Bool *)msgin;
  /*
   * Do something with your receive message
   */
  led_test_state = led_msg->data;
  digitalWrite(LED_PIN_TEST, led_test_state);

}
void int16array_callback(const void *msgin) {

  std_msgs__msg__Int16MultiArray * int16array_recv_msg = (std_msgs__msg__Int16MultiArray *)msgin;
  /*
   * Do something with your receive message
   */
  int16array_send_msg.data.data[0] = int16array_recv_msg->data.data[0];
  int16array_send_msg.data.data[1] = int16array_recv_msg->data.data[1];

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    /*
       TODO : Publish anything inside here
       
       For example, we are going to echo back
       the int16array_sub data to int16array_pub data,
       so we could see the data reflect each other.

       And also keep incrementing the int16_pub
    */
    rcl_ret_t ret = RCL_RET_OK;
    ret = rcl_publish(&int16array_pub, &int16array_send_msg, NULL);
    
    int16_msg.data++;
    ret = rcl_publish(&int16_pub, &int16_msg, NULL);
    

  }
}