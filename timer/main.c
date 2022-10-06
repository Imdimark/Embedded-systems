/*
 * File:   main.c
 * Author: giova
 *
 * Created on 6 ottobre 2022, 15.05
 * ------------pin a disposizione------------------
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
#include "xc.h"
#define TIMER1 1 
#define TIMER2 2
#define FOSc 7372800 //Hz
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void SetOnLed();
void SetOffLed();
void SetOnLed() { //It is also the first point
    LATBbits.LATB0 = 1; // set the pin high
}
void SetOffLed() {
    LATBbits.LATB0 = 0; // set the pin high
}

void tmr_setup_period(int timer, int ms){
    TMR1=0; //reset timer counter, appena raggi8unge i 500ms lo fa automaticamente
    T1CONbits.TON = 1; // starts the timer!
    
}
void tmr_wait_period(int timer){
    int flag = 0;
    while (flag = 1){
        if (IFS0bits.T1IF == 1){
            flag = 1;
            IFS0bits.T1IF //reset del flag
        }
    }
}

int main(void) {
    int Clock_Steps = (FOSc/4) * (timer/1000);
    // 7372800Hz/4 = 1.843.200 Hz // 1.843.200 Hz * 0.5 s = 921.600 Clock >> limite
    // Timer, has a prescale option of 1:1, 1:8, 1:64, and 1:256 00 01 «10» 11
    // 921.600 / 6
    PR1 =14.400; // (FOSc/4)/64 < limite
    T1CONbits.TCKPS = 2; // prescaler 1:64 
    TRISBbits.TRISB0 = 0; // set the pin as output for led
    while(1){
        tmr_setup_period(TIMER1, 500);
        SetOnLed();
        tmr_wait_period(TIMER1); //code + timer is 500. at the end it must restart the timer, every loop 500ms
        SetOffLed();
    }
    return 0;
}
