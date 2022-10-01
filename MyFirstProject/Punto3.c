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

// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <unistd.h>

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
//-------------------------------
void DelayTimer1Init(void) { // Set up timer 1 for microsecond delay function
    T1CON = 0x0000; // Timer off, 16-bit, 1:1 prescale, gating off
}
void UsDelay(unsigned int udelay) {
    TMR1 = 0;
    PR1 = udelay * 15; // Number of ticks per microsecond
    IFS0bits.T1IF = 0; // Reset interrupt flag
    T1CONbits.TON = 1;
    while (!IFS0bits.T1IF); // Wait here for timeout
    T1CONbits.TON = 0;
}
//-------------------------------
int main(void) {
    int flag = 0; //flag 0 means you have the possibility to change led's state
    TRISBbits.TRISB0 = 0; // set the pin as output for led
    TRISEbits.TRISE8 = 1; // set the pin as input for the button
    int oldstate;
    while(1){
        pressed = PORTEbits.RE8;
        if (pressed != oldstate) {
            flag == 0;  
        }
        if (flag == 0){
            TriggerOnOff();          
            flag = 1;   
        }      
        oldstate = pressed;
        UsDelay(1000);
    }     
    return 0;
}