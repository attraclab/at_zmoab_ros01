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

#include <ESP32Servo.h> 

#include "driver/uart.h"

#include <ModbusMaster.h>
#include <ZLAC8015D.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

///////////////////
/// Declaration ///
///////////////////

/*-------*/
/* Servo */
/*-------*/
Servo servo1;
Servo servo2;

int pwm_min = 700;
int pwm_mid = 1500;
int pwm_max = 2300;

#define SERVO1_PIN 7
#define SERVO2_PIN 15

#define PWM1_PIN 5
#define PWM2_PIN 6

struct {
  unsigned long down_stamp;
  unsigned long high_stamp;
  bool allow_cal_pwm = false;

  unsigned long last_log_stamp;
  unsigned long pwm_val;
  bool prev_pin_state = false;
  bool pin_state = false;
} pwm1_struct, pwm2_struct;

/* --------- */
/* SBUS UART */
/* --------- */

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

uint16_t ch5 = 1024;
uint16_t prev_ch5 = 1024;
int8_t cart_mode = 1;

/* ------------------------ */
/* ZLAC8015D & ModbusMaster */
/* ------------------------ */
#define MAX485_DE  2
#define MODBUS_RX_PIN 44
#define MODBUS_TX_PIN 43

ModbusMaster ModbusNode;
ZLAC8015D driver;
uint8_t res;

/* ---- */
/* Cart */
/* ---- */
int16_t rpmL;
int16_t rpmR;
float rpm[2];
float prev_y = 0.0;
float max_rpm = 200.0;
float rpmDB = 5.0;
int16_t rpmFB[2];
unsigned long last_recv_rpm_cmd_stamp;
int16_t faultCode[2] = {0,0};
unsigned long last_fault_code_stamp;
bool got_faultCode = false;
bool prev_got_faultCode = false;
int32_t encoderFB[2] = {0,0};

/* --- */
/* IMU */
/* --- */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool bno055_connected = false;
unsigned long last_bno055_check_stamp;
double qw, qx, qy, qz;
double ax, ay, az;
double gx, gy, gz;

/* --- */
/* LED */
/* --- */
#define AGENT_CONNECTED_LED_PIN 1
#define LED2_PIN 42
#define LED3_PIN 41
#define LED4_PIN 40 
bool led_test_state = false;
hw_timer_t *ledAgentConnected_timer = NULL;
bool agent_led_toggle = false;
bool agentConnectedState;

/* ----------------------- */
/* ROS State and Reconnect */
/* ----------------------- */
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

/* ----------------- */
/* Micro ROS declare */
/* ----------------- */

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t micros_node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t sbus_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t pwm_in_pub;
rcl_publisher_t cart_mode_pub;
rcl_publisher_t rpm_fb_pub;
rcl_publisher_t fault_code_pub;
rcl_publisher_t encoder_pub;
rcl_subscription_t servo_sub;
rcl_subscription_t rpm_cmd_sub;
rcl_subscription_t cart_mode_cmd_sub;
rcl_subscription_t disable_motor_sub;

std_msgs__msg__UInt16MultiArray sbus_msg;
std_msgs__msg__Int16MultiArray rpm_cmd_msg;
std_msgs__msg__UInt16MultiArray pwm_in_msg;
std_msgs__msg__Int16MultiArray rpm_fb_msg;
std_msgs__msg__Int16MultiArray fault_code_msg;
std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__Int8 cart_mode_msg;
std_msgs__msg__UInt16MultiArray servo_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int8 cart_mode_cmd_msg;
std_msgs__msg__Bool disable_motor_msg;

/////////////////////////
/// Interrupt Routine ///
/////////////////////////
void IRAM_ATTR readPWM1_ISR(){
  pwm1_struct.prev_pin_state = pwm1_struct.pin_state;
  pwm1_struct.pin_state = digitalRead(PWM1_PIN);
  if (pwm1_struct.pin_state == true){
    pwm1_struct.high_stamp = micros();
  } else if ((pwm1_struct.prev_pin_state != pwm1_struct.pin_state) && (pwm1_struct.pin_state == false)){
    pwm1_struct.down_stamp = micros();
    pwm1_struct.pwm_val = (pwm1_struct.down_stamp - pwm1_struct.high_stamp);
    pwm1_struct.allow_cal_pwm = true;
  }
}

void IRAM_ATTR readPWM2_ISR(){
  pwm2_struct.prev_pin_state = pwm2_struct.pin_state;
  pwm2_struct.pin_state = digitalRead(PWM2_PIN);
  if (pwm2_struct.pin_state == true){
    pwm2_struct.high_stamp = micros();
  } else if ((pwm2_struct.prev_pin_state != pwm2_struct.pin_state) && (pwm2_struct.pin_state == false)){
    pwm2_struct.down_stamp = micros();
    pwm2_struct.pwm_val = (pwm2_struct.down_stamp - pwm2_struct.high_stamp);
    pwm2_struct.allow_cal_pwm = true;
  }
}

void IRAM_ATTR ledAgentConnected_ISR(){
  if (agentConnectedState){
    agent_led_toggle = ! agent_led_toggle;
    digitalWrite(AGENT_CONNECTED_LED_PIN, !digitalRead(AGENT_CONNECTED_LED_PIN));
  }
  
}

/////////////////////
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
          
          prev_ch5 = ch5;

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

          ch5 = ch[4];

          /* Grab the lost frame */
          lost_frame = buf_[23] & 0x04;
          /* Grab the failsafe */
          failsafe = buf_[23] & 0x08;

          if ((lost_frame == true) || (failsafe == true)){
            init_sbus_val();
            cart_mode = 2;
          } else {
            if ((ch5 > 1500) && (prev_ch5 != ch5)){
              cart_mode = 2;
            } else if ((ch5 > 800) && (prev_ch5 != ch5)){
              cart_mode = 1;
            } else if ((ch5 < 800) && (prev_ch5 != ch5)) {
              cart_mode = 0;
            }
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

/////////////////////////
/// Helper Functions ///
////////////////////////
void preTransmission(){
  digitalWrite(MAX485_DE, 1);
}

void postTransmission(){
  digitalWrite(MAX485_DE, 0);
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

bool init_BNO055(){
  /// BNO055 setup//
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    return false;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  return true;
}

/////////////////////
/// ROS callbacks ///
/////////////////////
void rpm_cmd_callback(const void *msgin) {

  std_msgs__msg__Int16MultiArray * rpm_cmd_msg = (std_msgs__msg__Int16MultiArray *)msgin;

  if (cart_mode == 2){
    rpmL = rpm_cmd_msg->data.data[0];
    rpmR = rpm_cmd_msg->data.data[1];

    last_recv_rpm_cmd_stamp = millis();
    
  }
}
void servo_callback(const void *msgin) {

  std_msgs__msg__UInt16MultiArray * servo_msg = (std_msgs__msg__UInt16MultiArray *)msgin;

  servo1.writeMicroseconds(servo_msg->data.data[0]);
  servo2.writeMicroseconds(servo_msg->data.data[1]);
}
void cart_mode_cmd_callback(const void *msgin) {
  std_msgs__msg__Int8 * cart_mode_cmd_msg = (std_msgs__msg__Int8 *)msgin;

  cart_mode = cart_mode_cmd_msg->data;
}

void disable_motor_callback(const void *msgin) {
  std_msgs__msg__Bool * disable_motor_msg = (std_msgs__msg__Bool *)msgin;
  
  if (disable_motor_msg->data){
    res = driver.disable_motor();
  } else {
    res = driver.enable_motor();
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    rcl_ret_t ret = RCL_RET_OK;

    /// Publish Cart mode ///
    cart_mode_msg.data = cart_mode;
    ret = rcl_publish(&cart_mode_pub, &cart_mode_msg, NULL);

    /// Publish PWM Input ///
    pwm_in_msg.data.data[0] = (uint16_t)pwm1_struct.pwm_val;
    pwm_in_msg.data.data[1] = (uint16_t)pwm2_struct.pwm_val;
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
    rpm_fb_msg.data.data[0] = rpmFB[0];
    rpm_fb_msg.data.data[1] = rpmFB[1];
    ret = rcl_publish(&rpm_fb_pub, &rpm_fb_msg, NULL);

    /// Publish Fault Code ///
    fault_code_msg.data.data[0] = faultCode[0];
    fault_code_msg.data.data[1] = faultCode[1];
    ret = rcl_publish(&fault_code_pub, &fault_code_msg, NULL);

    /// Publish Encoder count ///
    encoder_msg.data.data[0] = encoderFB[0];
    encoder_msg.data.data[1] = encoderFB[1];
    ret = rcl_publish(&encoder_pub, &encoder_msg, NULL);
    

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
    &pwm_in_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/zmoab/pwm_in", &rmw_qos_profile_default);

  rclc_publisher_init(
    &sbus_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/zmoab/sbus_rc_ch", &rmw_qos_profile_default);

  rclc_publisher_init(
    &imu_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/zmoab/imu", &rmw_qos_profile_default);

  rclc_publisher_init(
    &cart_mode_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/zmoab/cart_mode", &rmw_qos_profile_default);

  rclc_publisher_init(
    &rpm_fb_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/zmoab/rpm_fb", &rmw_qos_profile_default);

  rclc_publisher_init(
    &fault_code_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/zmoab/fault_code", &rmw_qos_profile_default);

  rclc_publisher_init(
    &encoder_pub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/zmoab/encoder", &rmw_qos_profile_default);

  rclc_subscription_init(
    &rpm_cmd_sub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/zmoab/rpm_cmd", &rmw_qos_profile_default);

  rclc_subscription_init(
    &servo_sub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/zmoab/servo_cmd", &rmw_qos_profile_default);

  rclc_subscription_init(
    &cart_mode_cmd_sub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/zmoab/cart_mode_cmd", &rmw_qos_profile_default);

  rclc_subscription_init(
    &disable_motor_sub,
    &micros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/zmoab/disable_motor", &rmw_qos_profile_default);

  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   */
  unsigned int num_handles = 5;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &rpm_cmd_sub, &rpm_cmd_msg, &rpm_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &cart_mode_cmd_sub, &cart_mode_cmd_msg, &cart_mode_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &disable_motor_sub, &disable_motor_msg, &disable_motor_callback, ON_NEW_DATA);
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

  ret = rcl_publisher_fini(&pwm_in_pub, &micros_node);
  ret = rcl_publisher_fini(&cart_mode_pub, &micros_node);
  ret = rcl_publisher_fini(&imu_pub, &micros_node);
  ret = rcl_publisher_fini(&sbus_pub, &micros_node);
  ret = rcl_publisher_fini(&rpm_fb_pub, &micros_node);
  ret = rcl_publisher_fini(&fault_code_pub, &micros_node);
  ret = rcl_publisher_fini(&encoder_pub, &micros_node);
  ret = rcl_subscription_fini(&servo_sub, &micros_node);
  ret = rcl_subscription_fini(&rpm_cmd_sub, &micros_node);
  ret = rcl_subscription_fini(&cart_mode_cmd_sub, &micros_node);
  ret = rcl_subscription_fini(&disable_motor_sub, &micros_node);
}

/////////////////////
/// Arduino Setup ///
/////////////////////
void setup() {

  /* --------- */
  /* USB setup */ 
  /* --------- */
  USBSerial.begin(115200);

  /* --------------- */
  /* Micro ROS setup */ 
  /* --------------- */
  set_microros_serial_transports(USBSerial);
  delay(2000);

  state = WAITING_AGENT;
  prev_state = WAITING_AGENT;

  /* ---------- */
  /* GPIO setup */
  /* ---------- */
  pinMode(AGENT_CONNECTED_LED_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  digitalWrite(LED2_PIN, 0);
  digitalWrite(LED3_PIN, 0);
  digitalWrite(LED4_PIN, 0);

  /* LED Timer ISR */
  ledAgentConnected_timer = timerBegin(0, 80, true); // prescalar 80 -> Timer running at 1MHz
  timerAttachInterrupt(ledAgentConnected_timer, &ledAgentConnected_ISR, true);
  timerAlarmWrite(ledAgentConnected_timer, 50000, true); // specify second arg as how much time in microsec to execute ISR
  timerAlarmEnable(ledAgentConnected_timer);

  /* --------------- */
  /* PWM Servo setup */
  /* --------------- */
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(240);// Standard 50hz servo
  servo2.setPeriodHertz(240);// Standard 50hz servo

  servo1.attach(SERVO1_PIN, pwm_min, pwm_max);
  servo2.attach(SERVO2_PIN, pwm_min, pwm_max);

  pinMode(PWM1_PIN, INPUT);
  attachInterrupt(PWM1_PIN, readPWM1_ISR, CHANGE);
  pwm1_struct.last_log_stamp = millis();
  pinMode(PWM2_PIN, INPUT);
  attachInterrupt(PWM2_PIN, readPWM2_ISR, CHANGE);
  pwm2_struct.last_log_stamp = millis();

  /* --------------- */
  /* SBUS UART setup */
  /* --------------- */
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

  /* ---------------------- */
  /* ZLAC8015D ModbusMaster */
  /* ---------------------- */
  Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  delay(1000);
  ModbusNode.begin(1, Serial);
  ModbusNode.preTransmission(preTransmission);
  ModbusNode.postTransmission(postTransmission);
  driver.set_modbus(&ModbusNode);
  bool done = false;
  
  while (! done) {
    res = driver.disable_motor();
    if (res == 0) {
      done = true;
    }
  }
  res = driver.set_mode(3);
  res = driver.enable_motor();
  res = driver.set_accel_time(0, 0);
  res = driver.set_decel_time(0, 0);

  delay(1000);

  /* --------- */
  /* IMU setup */
  /* --------- */
  last_bno055_check_stamp = millis();
  bno055_connected = init_BNO055();

  /* ------------ */
  /* ROS Msg init */
  /* ------------ */
  uint16_t pwm_data[2] = {1500, 1500};
  pwm_in_msg.data.capacity = 2; // number of array length
  pwm_in_msg.data.size = 2;     // number of array length
  pwm_in_msg.data.data = pwm_data;

  sbus_msg.data.capacity = 16; // number of array length
  sbus_msg.data.size = 16;     // number of array length
  sbus_msg.data.data = sbus_ch;

  uint16_t servo_data[2] = {1500, 1500};
  servo_msg.data.capacity = 2; // number of array length
  servo_msg.data.size = 2;     // number of array length
  servo_msg.data.data = servo_data;

  int16_t zero_data[2] = {0, 0};
  rpm_cmd_msg.data.capacity = 2; // number of array length
  rpm_cmd_msg.data.size = 2;     // number of array length
  rpm_cmd_msg.data.data = zero_data;

  rpm_fb_msg.data.capacity = 2; // number of array length
  rpm_fb_msg.data.size = 2;     // number of array length
  rpm_fb_msg.data.data = zero_data;

  cart_mode_msg.data = cart_mode;

  cart_mode_cmd_msg.data = cart_mode;

  disable_motor_msg.data = false;

  imu_msg.header.frame_id.data = "imu_link";
  imu_msg.header.stamp.sec = int(millis() / 1000);
  imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());

  fault_code_msg.data.capacity = 2; // number of array length
  fault_code_msg.data.size = 2;     // number of array length
  fault_code_msg.data.data = faultCode;

  encoder_msg.data.capacity = 2; // number of array length
  encoder_msg.data.size = 2;     // number of array length
  encoder_msg.data.data = encoderFB;

}

////////////////////
/// Arduino Loop ///
////////////////////
void loop() {

  /* --------------------- */
  /* Reconnection Checking */
  /* --------------------- */
  prev_state = state;
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
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

  // if ((state == AGENT_CONNECTED) && (prev_state != state)) {
  //   digitalWrite(AGENT_CONNECTED_LED_PIN, 1);
  // } else if ((state != AGENT_CONNECTED) && (prev_state != state)) {
  //   digitalWrite(AGENT_CONNECTED_LED_PIN, 0);
  // }

  if (state == AGENT_CONNECTED){
    agentConnectedState = true;
  } else {
    agentConnectedState = false;
  }

  /* ------------------------ */
  /* Drive and Read ZLAC8015D */
  /* ------------------------ */
  if (cart_mode == 2){
    if ((millis() - last_recv_rpm_cmd_stamp) >= 1000){
      rpmL = 0;
      rpmR = 0;
      res = driver.set_rpm(rpmL, rpmR);
    } else {
      res = driver.set_rpm(rpmL, rpmR);
    }
    
  } else if (cart_mode == 1){
    memcpy(sbus_ch, ch, sizeof(ch));
    channelMixing(sbus_ch[0], sbus_ch[1], rpm);
    rpmL = (int16_t)rpm[0];
    rpmR = (int16_t)rpm[1];

    res = driver.set_rpm(rpmL, rpmR);
    
  } else {
    rpmL = 0;
    rpmR = 0;
    res = driver.set_rpm(rpmL, rpmR);
  }

  res = driver.get_rpm(rpmFB);
  res = driver.get_encoder_count(encoderFB);
  
  /* --------------- */
  /* Read BNO055 IMU */
  /* --------------- */
  sensors_event_t orientationData , angVelocityData , linearAccelData, accelerometerData;
  //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  imu::Quaternion quat = bno.getQuat();
  qw = quat.w();
  qx = quat.x();
  qy = quat.y();
  qz = quat.z();

  // ax = linearAccelData.acceleration.x;
  // ay = linearAccelData.acceleration.y;
  // az = linearAccelData.acceleration.z;

  ax = accelerometerData.acceleration.x;
  ay = accelerometerData.acceleration.y;
  az = accelerometerData.acceleration.z;

  gx = angVelocityData.gyro.x;
  gy = angVelocityData.gyro.y;
  gz = angVelocityData.gyro.z;

  /* -------------------- */
  /* Check ESC Fault Code */
  /* -------------------- */
  if ((millis() - last_fault_code_stamp) >= 0){
    res = driver.get_fault_code(faultCode);
    last_fault_code_stamp = millis();

    prev_got_faultCode = got_faultCode;
    if ((faultCode[0] != 0) || (faultCode[1] != 0)){
      // digitalWrite(LED2_PIN, 1);
      got_faultCode = true;
    } else {
      // digitalWrite(LED2_PIN, 0);
      got_faultCode = false;
    }

    if ((prev_got_faultCode != got_faultCode) && (got_faultCode == true)){
      digitalWrite(LED2_PIN, 1);
    } else if ((prev_got_faultCode != got_faultCode) && (got_faultCode == false)){
      digitalWrite(LED2_PIN, 0);
    }

  }

}

