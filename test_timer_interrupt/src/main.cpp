#include <Arduino.h>

hw_timer_t *My_timer = NULL;
#define LED_PIN 1

void IRAM_ATTR onTimer(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}


void setup() {
  pinMode(LED_PIN, OUTPUT);

  My_timer = timerBegin(0, 80, true); // prescalar 80 -> Timer running at 1MHz
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 50000, true); // specify second arg as how much time in microsec to execute ISR
  timerAlarmEnable(My_timer);

}

void loop() {

}

