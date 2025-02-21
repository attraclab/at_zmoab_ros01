#ifndef __SBUS_H__
#define __SBUS_H__

#include <Arduino.h>

/* --------- */
/* SBUS UART */
/* --------- */

extern uint16_t sbus_min;
extern uint16_t sbus_max;
extern uint16_t sbus_mid;
extern uint16_t sbus_db;
extern uint16_t sbus_min_db;
extern uint16_t sbus_max_db;

extern uint16_t ch[16];
extern uint16_t sbus_ch[16];
extern volatile int8_t cart_mode;

void setup_sbus();
void init_sbus_val();
void set_cart_mode(int8_t _cart_mode);
static void sbus_rx_task(void *pvParameters);

#endif