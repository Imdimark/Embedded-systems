/*
 * File:   main.c
 * Author: giova
 *
 * Created on 27 settembre 2022, 11.18
------------pin a disposizione------------------
 * switch s5 = pin 17 --> RE8
  
 * switch s6 = pin 23 --> RD0
  
 * led d3 = pin 2 --> RB0
  
 * led d4 = pin 3 --> RB1
  
 * scelgo witch s5 e led d3
 */
#include "xc.h"
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <p30F4011.h>
int pressed = 0; 

void SetOnLed();
void SetOffLed();
void TriggerOnOff();

void SetOnLed() { //It is also the first point
    LATBbits.LATB0 = 1; // set the pin high
}
void SetOffLed() {
    LATBbits.LATB0 = 0; // set the pin high
}

void TriggerOnOff() {
    if (pressed == 1){
        SetOnLed(); 
    }
    else if (pressed == 0){
        SetOffLed ();
    } 
}

int main(void) {
    int flag = 0; //flag 0 means you have the possibility to change led's state
    TRISBbits.TRISB0 = 0; // set the pin as output for led
    TRISEbits.TRISE3 = 1; // set the pin as input for the button
    int oldstate;
    while(1){
        pressed = PORTDbits.RD3;
        if (pressed != oldstate) {
            flag == 0;  
        }
        
        if (flag == 0){
            TriggerOnOff();          
            __delay_ms(10);
            flag = 1;   
        }      
        oldstate = pressed;
            __delay_ms(10);
    }     
    return 0;
}