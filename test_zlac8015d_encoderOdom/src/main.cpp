#include <Arduino.h>
#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include "driver/uart.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>


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
} state;

const int timeout_ms = 1000;

////////////
/// SBUS ///
////////////
#define SBUS_UART UART_NUM_1
#define SBUS_RX 18
#define SBUS_TX 17
#define BUF_SIZE (1024 * 2)
static QueueHandle_t uart1_queue;
uint16_t ch[16];
uint16_t checksum = 0;
uint16_t sbus_ch[16];
uint16_t sbus_min = 368;
uint16_t sbus_max = 1680;
uint16_t sbus_mid = 1024;
uint16_t sbus_db = 20;
uint16_t sbus_min_db = sbus_mid - sbus_db;
uint16_t sbus_max_db = sbus_mid + sbus_db;
static const char * TAG = "";
bool lost_frame;
bool failsafe;
bool got_sbus_data = false;

////////////////////
/// SBUS Threads ///
////////////////////
void init_sbus_val(){
  for (int i=0; i<16; i++){
    ch[i] = 1024;
    sbus_ch[i] = 1024;
  }
}
static void sbus_rx_task(void *pvParameters)
{
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;

  //Infinite loop to run main bulk of task
  while (1) {

    //Loop will continually block (i.e. wait) on event messages from the event queue
    if (xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {

      //Handle received event
      if (event.type == UART_DATA) {

        uint8_t buf_[128];
        int SBUS_data_length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(SBUS_UART, (size_t*)&SBUS_data_length));
        SBUS_data_length = uart_read_bytes(SBUS_UART, buf_, SBUS_data_length, 100);

        // from https://github.com/bolderflight/sbus/tree/main //
        if (buf_[0] == 0x0F) {
          ch[0]  = static_cast<uint16_t>(buf_[1] | ((buf_[2] << 8) & 0x07FF));
          ch[1]  = static_cast<uint16_t>((buf_[2] >> 3) | ((buf_[3] << 5) & 0x07FF));
          ch[2]  = static_cast<uint16_t>((buf_[3] >> 6) | (buf_[4] << 2) | ((buf_[5] << 10) & 0x07FF));
          ch[3]  = static_cast<uint16_t>((buf_[5] >> 1) | ((buf_[6] << 7) & 0x07FF));
          ch[4]  = static_cast<uint16_t>((buf_[6] >> 4) | ((buf_[7] << 4) & 0x07FF));
          ch[5]  = static_cast<uint16_t>((buf_[7] >> 7) | (buf_[8] << 1) | ((buf_[9] << 9) & 0x07FF));
          ch[6]  = static_cast<uint16_t>((buf_[9] >> 2) | ((buf_[10] << 6) & 0x07FF));
          ch[7]  = static_cast<uint16_t>((buf_[10] >> 5) | ((buf_[11] << 3) & 0x07FF));
          ch[8]  = static_cast<uint16_t>(buf_[12] | ((buf_[13] << 8) & 0x07FF));
          ch[9]  = static_cast<uint16_t>((buf_[13] >> 3) | ((buf_[14] << 5) & 0x07FF));
          ch[10] = static_cast<uint16_t>((buf_[14] >> 6) | (buf_[15] << 2) | ((buf_[16] << 10) & 0x07FF));
          ch[11] = static_cast<uint16_t>((buf_[16] >> 1) | ((buf_[17] << 7) & 0x07FF));
          ch[12] = static_cast<uint16_t>((buf_[17] >> 4) | ((buf_[18] << 4) & 0x07FF));
          ch[13] = static_cast<uint16_t>((buf_[18] >> 7) | (buf_[19] << 1) | ((buf_[20] << 9) & 0x07FF));
          ch[14] = static_cast<uint16_t>((buf_[20] >> 2) | ((buf_[21] << 6) & 0x07FF));
          ch[15] = static_cast<uint16_t>((buf_[21] >> 5) | ((buf_[22] << 3) & 0x07FF));

          /* Grab the lost frame */
          lost_frame = buf_[23] & 0x04;
          /* Grab the failsafe */
          failsafe = buf_[23] & 0x08;

          if ((lost_frame == true) || (failsafe == true)){
            init_sbus_val();
          }

          got_sbus_data = true;
        }
      }
    }
    //If you want to break out of the loop due to certain conditions, set exit condition to true
    if (exit_condition) {
      break;
    }
  }
  //Out side of loop now. Task needs to clean up and self terminate before returning
  vTaskDelete(NULL);
}

//////////////////
/// BNO055 IMU ///
//////////////////
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool bno055_connected = false; 

double qw, qx, qy, qz;
double ax, ay, az;
double gx, gy, gz;

bool init_BNO055(){
  /// BNO055 setup//
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    //USBSerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  return true;
}

////////////////////////////////
/// ZLAC8015D & ModbusMaster ///
////////////////////////////////
#define MAX485_DE  2
#define MODBUS_RX_PIN 44
#define MODBUS_TX_PIN 43

void preTransmission(){
  digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
  digitalWrite(MAX485_DE, 0);
}

ModbusMaster node;
ZLAC8015D driver;
uint8_t res;

///////////////////
/// Cart Mixing ///
///////////////////
int16_t rpmL;
int16_t rpmR;
float rpm[2];
float prev_y = 0.0;
float max_rpm = 200.0;
float rpmDB = 5.0;
int16_t rpmFB[2];
int32_t encoderFB[2];

////////////////
/// Odometry ///
////////////////
/* ref. https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/ */
hw_timer_t *odom_timer = NULL;
#define odom_refreshRate_microsec 20000 //20ms 50Hz
#define wheel_circum 0.656
#define countPerRound 16538.0 // 16400
#define distInOneCount wheel_circum/countPerRound //0.656/16538  4.00366E-5
#define PI 3.141592653 //22/7
#define R_wheel wheel_circum/(2*PI) //0.1044056
#define wheel_base 0.35 
double x = 0.0;
double y = 0.0;
double the = 0.0;
double x_prev = 0.0;
double y_prev = 0.0;
double vel_x = 0.0;
double vel_y = 0.0;
double v_L = 0.0;
double v_R = 0.0;
unsigned long last_vel_cal_stamp = millis();
double diff_t = 0.0;
double the_prev = 0.0;
double diff_x = 0.0; 
double diff_y = 0.0;
double D_L = 0.0;
double D_R = 0.0;
double D_ave = 0.0;
double diff_the = 0.0;
double encLeft_diff = 0.0;
double encRight_diff = 0.0;
int32_t encLeft_now = 0;
int32_t encRight_now = 0;
int32_t encLeft_prev = 0;
int32_t encRight_prev = 0;
bool got_encoder_read = false;
bool got_first_count = false;
double rpy[3] = {0.0, 0.0, 0.0};
double quat[4] = {0.0, 0.0, 0.0, 1.0};
double wheel_travel = 0.0;
double vel_the = 0.0;
float cmd_vel[2] = {0.0, 0.0};
double x2 = 0.0;
double y2 = 0.0;
double the2 = 0.0;
double x2_prev;
double y2_prev;
double the2_prev;
double rpy2[3] = {0.0, 0.0, 0.0};
double quat2[4] = {0.0, 0.0, 0.0, 1.0};
double R_icc;

unsigned long start_time;
unsigned long period;
unsigned long last_print_stamp = millis();
unsigned long last_ISR_stamp = millis();

void reset_odom_variables(){
  x = 0.0;
  y = 0.0;
  the = 0.0;
  x_prev = 0.0;
  y_prev = 0.0;
  vel_x = 0.0;
  vel_y = 0.0;
  v_L = 0.0;
  v_R = 0.0;
  last_vel_cal_stamp = millis();
  diff_t = 0.0;
  the_prev = 0.0;
  diff_x = 0.0; 
  diff_y = 0.0;
  D_L = 0.0;
  D_R = 0.0;
  D_ave = 0.0;
  diff_the = 0.0;
  encLeft_diff = 0.0;
  encRight_diff = 0.0;
  encLeft_now = 0;
  encRight_now = 0;
  encLeft_prev = 0;
  encRight_prev = 0;
  got_encoder_read = false;
  got_first_count = false;
  rpy[0] = 0.0;
  rpy[1] = 0.0;
  rpy[2] = 0.0;
  quat[0] = 0.0;
  quat[1] = 0.0;
  quat[2] = 0.0;
  quat[3] = 1.0;
  vel_the = 0.0;
  cmd_vel[0] = 0.0;
  cmd_vel[1] = 0.0;
  x2 = 0.0;
  y2 = 0.0;
  the2 = 0.0;
  x2_prev = 0.0;
  y2_prev = 0.0;
  the2_prev = 0.0;
  rpy2[0] = 0.0;
  rpy2[1] = 0.0;
  rpy2[2] = 0.0;
  quat2[0] = 0.0;
  quat2[1] = 0.0;
  quat2[2] = 0.0;
  quat2[3] = 1.0;
  R_icc = 0.0;
}

/////////////////
/// Micro ROS ///
/////////////////
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t micros_node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t odom_pub;
rcl_subscription_t odom_reset_sub;
rcl_publisher_t imu_pub;
rcl_publisher_t cmd_vel_pub;
rcl_publisher_t odom2_pub;
rcl_publisher_t rpmFB_pub;

nav_msgs__msg__Odometry odom_msg;
nav_msgs__msg__Odometry odom2_msg;
std_msgs__msg__Bool odom_reset_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray cmd_vel_msg;
std_msgs__msg__Int16MultiArray rpmFB_msg;

void quaternion_from_euler(double _rpy[3], double _quat[4]){
  double ci, si, cj, sj, ck, sk, cc, cs, sc, ss;
  double roll = _rpy[0];
  double pitch = _rpy[1];
  double yaw = _rpy[2];

  ci = cos(roll*0.5);
  si = sin(roll*0.5);
  cj = cos(pitch*0.5);
  sj = sin(pitch*0.5);
  ck = cos(yaw*0.5);
  sk = sin(yaw*0.5);
  cc = ci*ck;
  cs = ci*sk;
  sc = si*ck;
  ss = si*sk;

  _quat[0] = cj*sc - sj*cs; // qx
  _quat[1] = cj*ss + sj*cc; // qy
  _quat[2] = cj*cs - sj*sc; // qz
  _quat[3] = cj*cc + sj*ss; // qw

}
void odom_reset_callback(const void *msgin) {

  std_msgs__msg__Bool * odom_reset_msg = (std_msgs__msg__Bool *)msgin;
  
  reset_odom_variables();
  bno055_connected = init_BNO055();

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
    int64_t time_ns = rmw_uros_epoch_nanos();

    rcl_ret_t ret = RCL_RET_OK;
    odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000); //RCL_MS_TO_NS(millis());
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    rpy[2] = the;
    quaternion_from_euler(rpy, quat);
    odom_msg.pose.pose.orientation.x = quat[0];
    odom_msg.pose.pose.orientation.y = quat[1];
    odom_msg.pose.pose.orientation.z = quat[2];
    odom_msg.pose.pose.orientation.w = quat[3];

    odom_msg.twist.twist.linear.x = vel_x;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = vel_the;
    
    ret = rcl_publish(&odom_pub, &odom_msg, NULL);

    /// Publish IMU data ///
    imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000); //int(millis() / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000); //RCL_MS_TO_NS(millis());
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

    /// Publish cmd_vel ///
    cmd_vel_msg.data.data[0] = cmd_vel[0];
    cmd_vel_msg.data.data[1] = cmd_vel[1];
    ret = rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL);

    /// Odom2 ///
    odom2_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    odom2_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000); //RCL_MS_TO_NS(millis());
    odom2_msg.pose.pose.position.x = x2;
    odom2_msg.pose.pose.position.y = y2;
    rpy2[2] = the2;
    quaternion_from_euler(rpy2, quat2);
    odom2_msg.pose.pose.orientation.x = quat2[0];
    odom2_msg.pose.pose.orientation.y = quat2[1];
    odom2_msg.pose.pose.orientation.z = quat2[2];
    odom2_msg.pose.pose.orientation.w = quat2[3];

    odom2_msg.twist.twist.linear.x = vel_x;
    odom2_msg.twist.twist.linear.y = 0.0;
    odom2_msg.twist.twist.angular.z = vel_the;
    
    ret = rcl_publish(&odom2_pub, &odom2_msg, NULL);

    /// Publish RPM FB ///
    rpmFB_msg.data.data[0] = rpmFB[0];
    rpmFB_msg.data.data[1] = rpmFB[1];
    ret = rcl_publish(&rpmFB_pub, &rpmFB_msg, NULL);
    

  }
}

bool create_entities(){
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  rmw_uros_sync_session(timeout_ms);

  const char * node_name = "esp32_odom_node";
  const char * ns = "";
  const int domain_id = 0;
  
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = RCL_RET_OK;
  ret = rcl_init_options_init(&init_options, allocator);
  ret = rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&micros_node, node_name, ns, &support);

  rclc_publisher_init(
    &odom_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/zmoab/raw_odom", &rmw_qos_profile_default);

  rclc_publisher_init(
    &odom2_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/zmoab/raw_odom2", &rmw_qos_profile_default);

  rclc_publisher_init(
    &imu_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/zmoab/imu", &rmw_qos_profile_default);

  rclc_publisher_init(
    &cmd_vel_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/zmoab/cmd_vel", &rmw_qos_profile_default);

  rclc_publisher_init(
    &rpmFB_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/zmoab/rpm_fb", &rmw_qos_profile_default);

  rclc_subscription_init(
    &odom_reset_sub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/zmoab/odom_reset", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  unsigned int num_handles = 2;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &odom_reset_sub, &odom_reset_msg, &odom_reset_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}
/*
 * Clean up all the created objects
 */
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

  ret = rcl_publisher_fini(&odom_pub, &micros_node);
  ret = rcl_publisher_fini(&odom2_pub, &micros_node);
  ret = rcl_publisher_fini(&imu_pub, &micros_node);
  ret = rcl_publisher_fini(&cmd_vel_pub, &micros_node);
  ret = rcl_publisher_fini(&rpmFB_pub, &micros_node);
  ret = rcl_subscription_fini(&odom_reset_sub, &micros_node);
}



void channelMixing(uint16_t str_ch, uint16_t thr_ch, float _rpm[2]){
  
  float y;
  float x;

  if (thr_ch >= sbus_max_db){
    y = (float)map(thr_ch, sbus_max_db, sbus_max, 0.0, 100.0);
  } else if (thr_ch <= sbus_min_db){
    y = (float)map(thr_ch, sbus_min, sbus_min_db, -100.0, 0.0);
  } else {
    y = 0.0;
  }

  if (str_ch >= sbus_max_db){
    x = (float)map(str_ch, sbus_max_db, sbus_max, 0.0, 100.0);
  } else if (str_ch <= sbus_min_db){
    x = (float)map(str_ch, sbus_min, sbus_min_db, -100.0, 0.0);
  } else {
    x = 0.0;
  }
  


  float left, right;

  left = y + x;
  right = y - x;

  float diff = abs(x) - abs(y);

  if (left < 0.0){
    left = left - abs(diff);
  } else {
    left = left + abs(diff);
  }

  if (right < 0.0){
    right = right - abs(diff);
  } else {
    right = right + abs(diff);
  }

  if (prev_y < 0.0){
    float swap;
    swap = left;
    left = right;
    right = swap;
  }

  prev_y = y;

  

  _rpm[0] = map(left, -200.0, 200.0, -max_rpm, max_rpm);
  _rpm[1] = map(right, -200.0, 200.0, -max_rpm, max_rpm);

  if ((_rpm[0] < rpmDB) && (_rpm[0] > -rpmDB)){
    _rpm[0] = 0.0;
  }
  if ((_rpm[1] < rpmDB) && (_rpm[1] > -rpmDB)){
    _rpm[1] = 0.0;
  }
  
}

void rpm_To_velCommand(int16_t rpmL, int16_t rpmR, float _cmd_vel[2]){
  float vl, vr, Vx, Wz;
  vl = (float)rpmL * (2.0*PI/60.0) * R_wheel;
  vr = (float)rpmR * (2.0*PI/60.0) * R_wheel;
  Vx = (vl + vr)*0.5;
  Wz = (vr - vl)/wheel_base;
  _cmd_vel[0] = Vx;
  _cmd_vel[1] = Wz;
}

void IRAM_ATTR odomComputeISR(){

  if (got_encoder_read){

    // Displacement update //
    encLeft_diff = (double)(encLeft_now - encLeft_prev);
    encRight_diff = (double)(encRight_now - encRight_prev);
    D_L = (encLeft_diff)*distInOneCount;
    D_R = (encRight_diff)*distInOneCount;
    D_ave = (D_L + D_R)/2.0;
    diff_the = (D_R - D_L)/wheel_base;
    encLeft_prev = encLeft_now;
    encRight_prev = encRight_now;

    // Odometry update //
    the_prev = the;
    the = the_prev + diff_the;
    diff_x = D_ave*cos(the);
    diff_y = D_ave*sin(the);
    x_prev = x;
    y_prev = y;
    x = x_prev + diff_x;
    y = y_prev + diff_y;

    diff_t = millis() - last_vel_cal_stamp;

    if (diff_t < 1000.0) {

      v_L = rpmFB[0] * (2*PI/60.0) * R_wheel;
      v_R = rpmFB[1] * (2*PI/60.0) * R_wheel;
      vel_x = (v_L + v_R)/2.0;
      vel_the = (v_R - v_L)/wheel_base;

      // /// Straight line ///
      // // if ((cmd_vel[0] != 0.0) && (cmd_vel[1] == 0.0)){
      // if ((vel_x != 0.0) && (abs(vel_the) < 0.1)){
      //   x2 = x2_prev + (vel_x*cos(the2)*(diff_t/1000.0));
      //   y2 = y2_prev + (vel_x*sin(the2)*(diff_t/1000.0));
      //   the2 = the2_prev;
      // } 
      // /// Turning In-Place ///
      // // else if ((cmd_vel[0] == 0.0) && (cmd_vel[1] != 0.0 )){
      // else if (((v_L > 0.0) && (v_R < 0.0)) || ((v_L < 0.0) && (v_R > 0.0))){
      //   the2 = the2_prev + vel_the*(diff_t/1000.0);
      // }
      // /// Curving 
      // else if ((v_L < v_R) || (v_L > v_R)){
        
      //   R_icc = (wheel_base/2.0) * ((v_R+v_L)/(v_R-v_L));
      //   x2 = x2_prev - R_icc*sin(the2) + R_icc*sin(the2 + (vel_the*(diff_t/1000.0)));
      //   y2 = y2_prev + R_icc*cos(the2) - R_icc*cos(the2 + (vel_the*(diff_t/1000.0)));
      //   the2 = the2_prev + (vel_the*(diff_t/1000.0));
      // }
      
      // x2_prev = x2;
      // y2_prev = y2;
      // the2_prev = the2;
    
    }
    
    last_vel_cal_stamp = millis();
    last_ISR_stamp = millis();

  }

}

void setup() {

  USBSerial.begin(115200);

  set_microros_serial_transports(USBSerial);
  delay(2000);

  ////////////
  /// SBUS ///
  ////////////
  uart_config_t uart1_config = {
    .baud_rate = 100000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  uart_param_config(SBUS_UART, &uart1_config);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  uart_set_pin(SBUS_UART, SBUS_TX, SBUS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(SBUS_UART, UART_SIGNAL_RXD_INV);
  uart_driver_install(SBUS_UART, BUF_SIZE, BUF_SIZE, 20, &uart1_queue, 0);

  init_sbus_val();

  xTaskCreate(sbus_rx_task, "sbus_rx_task", 2048, NULL, 12, NULL);

  //////////////////////////////
  /// ZLAC8015D ModbusMaster ///
  //////////////////////////////
  Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  delay(1000);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  driver.set_modbus(&node);
  bool done = false;
  
  while (! done) {
    res = driver.disable_motor();
    USBSerial.print("disable motor ");
    USBSerial.println(res);
    if (res == 0) {
      done = true;
    }
  }
  delay(1000);
  res = driver.set_mode(3);
  res = driver.enable_motor();
  res = driver.set_accel_time(0, 0);
  res = driver.set_decel_time(0, 0);

  delay(1000);

  //////////////////////
  /// Odom Timer ISR ///
  //////////////////////
  odom_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(odom_timer, &odomComputeISR, true);
  timerAlarmWrite(odom_timer, odom_refreshRate_microsec, true); // 20ms or 50Hz rate
  timerAlarmEnable(odom_timer);

  /* --------- */
  /* IMU setup */
  /* --------- */
  // last_bno055_check_stamp = millis();
  bno055_connected = init_BNO055();

  /////////////////////
  /// Micro ROS msg ///
  /////////////////////
  
  int64_t time_ns = rmw_uros_epoch_nanos();

  odom_msg.header.frame_id.data = "odom";
  odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000); //int(millis() / 1000);
  odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);; //RCL_MS_TO_NS(millis());
  odom_msg.child_frame_id.data = "base_link";
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;

  for (int i=0; i<36; i++){
    odom_msg.pose.covariance[i] = 0.0;
  }
  odom_msg.pose.covariance[0] = 0.0001;
  odom_msg.pose.covariance[7] = 0.0001;
  odom_msg.pose.covariance[14] = 0.000001;
  odom_msg.pose.covariance[21] = 0.000001;
  odom_msg.pose.covariance[28] = 0.000001;
  odom_msg.pose.covariance[35] = 0.0001;

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;
  for (int i=0; i<36; i++){
    odom_msg.twist.covariance[i] = 0.0;
  }


  odom_reset_msg.data = false;

  imu_msg.header.frame_id.data = "imu_link";
  imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000); //int(millis() / 1000);
  imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);; //RCL_MS_TO_NS(millis());

  float rpm_zero[2] = {0.0, 0.0};
  cmd_vel_msg.data.capacity = 2; // number of array length
  cmd_vel_msg.data.size = 2;     // number of array length
  cmd_vel_msg.data.data = rpm_zero;


  odom2_msg.header.frame_id.data = "odom";
  odom2_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000); //int(millis() / 1000);
  odom2_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);; //RCL_MS_TO_NS(millis());
  odom2_msg.child_frame_id.data = "base_link2";
  odom2_msg.pose.pose.position.x = 0.0;
  odom2_msg.pose.pose.position.y = 0.0;
  odom2_msg.pose.pose.position.z = 0.0;
  odom2_msg.pose.pose.orientation.x = 0.0;
  odom2_msg.pose.pose.orientation.y = 0.0;
  odom2_msg.pose.pose.orientation.z = 0.0;
  odom2_msg.pose.pose.orientation.w = 1.0;

  for (int i=0; i<36; i++){
    odom2_msg.pose.covariance[i] = 0.0;
  }
  odom2_msg.pose.covariance[0] = 0.0001;
  odom2_msg.pose.covariance[7] = 0.0001;
  odom2_msg.pose.covariance[14] = 0.000001;
  odom2_msg.pose.covariance[21] = 0.000001;
  odom2_msg.pose.covariance[28] = 0.000001;
  odom2_msg.pose.covariance[35] = 0.0001;

  odom2_msg.twist.twist.linear.x = 0.0;
  odom2_msg.twist.twist.linear.y = 0.0;
  odom2_msg.twist.twist.linear.z = 0.0;
  odom2_msg.twist.twist.angular.x = 0.0;
  odom2_msg.twist.twist.angular.y = 0.0;
  odom2_msg.twist.twist.angular.z = 0.0;
  for (int i=0; i<36; i++){
    odom2_msg.twist.covariance[i] = 0.0;
  }

  int16_t rpmFB_zero[2] = {0, 0};
  rpmFB_msg.data.capacity = 2; // number of array length
  rpmFB_msg.data.size = 2;     // number of array length
  rpmFB_msg.data.data = rpmFB_zero;
  

}

void loop() {

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (got_sbus_data){
    // start_time = millis();
    memcpy(sbus_ch, ch, sizeof(ch));

    channelMixing(sbus_ch[0], sbus_ch[1], rpm);
    rpmL = (int16_t)rpm[0];
    rpmR = (int16_t)rpm[1];

    res = driver.set_rpm(rpmL, rpmR);
    rpm_To_velCommand(rpmL, rpmR, cmd_vel);

    // res = driver.set_rpm(60, 60);
    res = driver.get_rpm(rpmFB);
    res = driver.get_encoder_count(encoderFB);
    encLeft_now = encoderFB[0];
    encRight_now = -encoderFB[1];

    if (! got_first_count){
      encLeft_prev =  encoderFB[0];
      encRight_prev = -encoderFB[1];
      got_first_count = true;
      got_encoder_read = true;
      //USBSerial.println("Got first count");
    }

    
    /* --------------- */
    /* Read BNO055 IMU */
    /* --------------- */
    sensors_event_t orientationData , angVelocityData , linearAccelData;
    //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    imu::Quaternion imu_quat = bno.getQuat();
    qw = imu_quat.w();
    qx = imu_quat.x();
    qy = imu_quat.y();
    qz = imu_quat.z();

    ax = linearAccelData.acceleration.x;
    ay = linearAccelData.acceleration.y;
    az = linearAccelData.acceleration.z;

    gx = angVelocityData.gyro.x;
    gy = angVelocityData.gyro.y;
    gz = angVelocityData.gyro.z;


    // period = millis() - start_time;
    /// It took ~13-14 ms 
    if (((millis() - last_print_stamp) > 100) && got_encoder_read && false){
      // USBSerial.println("encL: " + String(encoderFB[0]) + " encR: " + String(encoderFB[1]));
      // USBSerial.println("enL_di: " + String(encLeft_diff) + " enL_prev: " + String(encLeft_prev) + " enL_now: " + String(encLeft_now) + " enR_di: " + String(encRight_diff) + " enR_prev: " + String(encRight_prev)  + " enR_now: " + String(encRight_now));
      USBSerial.println("x: " + String(x) + " y: " + String(y) + " the: " + String(the*180/PI) + " vel_x: " + String(vel_x) + " diff_t: " + String(diff_t) + 
                        " DL " + String(D_L) + " DR " + String(D_R) +  
                          " enL_di: " + String(encLeft_diff) + " enL_prev: " + String(encLeft_prev) + " enL_now: " + String(encLeft_now) + 
                          " enR_di: " + String(encRight_diff) + " enR_prev: " + String(encRight_prev)  + " enR_now: " + String(encRight_now));
      //USBSerial.println("enL_di: " + String(encLeft_diff) + " enR_di: " + String(encRight_diff) + " DL: " + String(D_L) + " DR: " + String(D_R) + " Dave: " + String(D_ave) + " t: " + String(last_ISR_stamp));
      last_print_stamp = millis();
    }
    
  }
}

