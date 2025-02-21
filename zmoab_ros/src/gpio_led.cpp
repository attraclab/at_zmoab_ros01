
#include "gpio_led.h"
#include "ros_com.h"

#define AGENT_CONNECTED_LED_PIN 1
#define LED2_PIN 42
#define LED3_PIN 41
#define LED4_PIN 40 

bool led_test_state = false;
hw_timer_t *ledAgentConnected_timer = NULL;
bool agent_led_toggle = false;
bool agentConnectedState;

void setup_led(){
	pinMode(AGENT_CONNECTED_LED_PIN, OUTPUT);
	pinMode(LED2_PIN, OUTPUT);
	pinMode(LED3_PIN, OUTPUT);
	pinMode(LED4_PIN, OUTPUT);

	digitalWrite(LED2_PIN, 1);
	digitalWrite(LED3_PIN, 1);
	digitalWrite(LED4_PIN, 1);

	ledAgentConnected_timer = timerBegin(0, 80, true); // prescalar 80 -> Timer running at 1MHz
	timerAttachInterrupt(ledAgentConnected_timer, &ledAgentConnected_ISR, true);
	timerAlarmWrite(ledAgentConnected_timer, 50000, true); // specify second arg as how much time in microsec to execute ISR
	timerAlarmEnable(ledAgentConnected_timer);
}

void IRAM_ATTR ledAgentConnected_ISR(){
	if (agentConnectedState){
		agent_led_toggle = ! agent_led_toggle;
		digitalWrite(AGENT_CONNECTED_LED_PIN, agent_led_toggle);
	} else {
		digitalWrite(AGENT_CONNECTED_LED_PIN, 1);
	}
  
}

void set_boardLED_byId(uint8_t _id, bool flag){

	if (_id == 2){
		digitalWrite(LED2_PIN, ! flag);
	} else if (_id == 3){
		digitalWrite(LED3_PIN, ! flag);
	} else if (_id == 4){
		digitalWrite(LED4_PIN, ! flag);
	}
}


void led_loop(){
	if (state_connected){
		agentConnectedState = true;
	} else {
		agentConnectedState = false;
	}
}