#include <Arduino.h>
#include "driver/uart.h"
#include <zlacd8015.h>
#include <ModbusRTU.h>

#define MAX485_DE  2
#define MODBUS_RX_PIN 43 //44
#define MODBUS_TX_PIN 44 //43

uint8_t res;
zlac8015d driver;
ModbusRTU mb;
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
bool first_drive = false;

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

          // USBSerial.print("ch1: ");
          // USBSerial.print(ch[0]);
          // USBSerial.print(" ch2: ");
          // USBSerial.println(ch[1]);

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





void setup() {

    USBSerial.begin();

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

    /// Modbus ///
    Serial.begin(115200, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    mb.begin(&Serial)
    driver.set_modbus(&Serial, MAX485_DE);
    

    do {
        res = driver.disable_motor();
        USBSerial.printf("disable motor %d \n", res);
    } while (res != 0);

    do {
        res = driver.set_mode(3);
        USBSerial.printf("set mode %d \n", res);
    } while (res != 0);

    do {
        res = driver.enable_motor();
        USBSerial.printf("enable motor %d \n", res);
    } while (res != 0);

    do {
        res = driver.set_accel_time(20, 20);
        USBSerial.printf("set accel %d \n", res);
    } while (res != 0);

    do {
        res = driver.set_decel_time(20, 20);
        USBSerial.printf("set decel %d \n", res);
    } while (res != 0);
}

void loop() {

    if (got_sbus_data){

        memcpy(sbus_ch, ch, sizeof(ch));
    
        channelMixing(sbus_ch[0], sbus_ch[1], rpm);
        rpmL = (int16_t)rpm[0];
        rpmR = (int16_t)rpm[1];
    
    
        res = driver.set_rpm(rpmL, rpmR);
        // USBSerial.printf("set rpm %d \n", res);
        
        // res = driver.get_rpm(rpmFB);
    
        // USBSerial.print("ch1: ");
        // USBSerial.print(sbus_ch[0]);
        // USBSerial.print(" ch2: ");
        // USBSerial.print(sbus_ch[1]);
        // USBSerial.print(" rpmL_cmd: ");
        // USBSerial.print(rpmL);
        // USBSerial.print(" rpmR_cmd: ");
        // USBSerial.print(rpmR);
        // USBSerial.print(" rpmL_fb: ");
        // USBSerial.print(rpmFB[0]);
        // USBSerial.print(" rpmR_fb: ");
        // USBSerial.println(rpmFB[1]);
        
        delay(10);
    
      }

}


