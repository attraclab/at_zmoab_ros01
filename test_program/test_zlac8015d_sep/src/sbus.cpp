

#include "sbus.h"
#include "driver/uart.h"

#define SBUS_UART UART_NUM_1
#define SBUS_RX 18
#define SBUS_TX 17
#define BUF_SIZE (1024 * 2)

static QueueHandle_t uart1_queue;

uint16_t checksum = 0;

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

volatile int8_t cart_mode = 1;
uint16_t ch[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t sbus_ch[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


void setup_sbus(){
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
}

void init_sbus_val(){
  for (int i=0; i<16; i++){
    ch[i] = 1024;
    sbus_ch[i] = 1024;
  }
}

void set_cart_mode(int8_t _cart_mode){
    cart_mode = _cart_mode;
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
