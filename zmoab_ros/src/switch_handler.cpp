#include <Arduino.h>

#define SW1 10
#define SW2 11
#define SW3 12
#define SW4 13

int8_t sw_val_list[4] = {0, 0, 0, 0};

void setup_switch(){

    pinMode(SW1, INPUT);
    pinMode(SW2, INPUT);
    pinMode(SW3, INPUT);
    pinMode(SW4, INPUT);

}

bool get_switch_value(int _SW){
    if (_SW == 1){
        return digitalRead(SW1);
    } else if (_SW == 2){
        return digitalRead(SW2);
    } else if (_SW == 3){
        return digitalRead(SW3);
    } else if (_SW == 4){
        return digitalRead(SW4);
    } else {
        return false;
    }
}

void switch_loop(){

    for (int i=0; i<4; i++){
        if (get_switch_value(i+1) == true){
            sw_val_list[i] = 0;
        } else {
            sw_val_list[i] = 1;
        }
        
    }

}