/*
 * File:   ClassWork 1.c
 * Author: youss
 *
 * Created on September 28, 2022, 8:39 PM
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
#include "xc.h"
#define TIMER1 1
#define TIMER2 2
#define delay 1000
#define HIGH 1
#define LOW 0
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
int main(void) {
    //TRISDbits.TRISD0 = 1; // set the pin as input "button S6"
    //TRISEbits.TRISE8 = 1; // set the pin as input "button S5"
    TRISBbits.TRISB0 = 0; // set the pin as output "led D3"
    tmr_setup_period(TIMER1, delay);
    while(1){
        LATBbits.LATB0 = HIGH;
        tmr_wait_period(TIMER1);
        LATBbits.LATB0 = LOW;
        tmr_wait_period(TIMER1);
    }
    return 0;
}

void tmr_setup_period(int timer, int ms){
    if(timer == 1){
        TMR1 = 0; //reset timer counter
        PR1= (ms *7.3728 *1000)/(4 *64);
        T1CONbits.TCKPS = 2; //prescaler 1:64
    }
    else if (timer == 2){
        TMR2 = 0; //reset timer counter
        PR2= (ms *7.3728 *1000)/(4 *64);
        T2CONbits.TCKPS = 2; //prescaler 1:64
    }
}
void tmr_wait_period(int timer){
    if(timer == 1){
        T1CONbits.TON = 1; // starts the timer!
    }
    else if (timer == 2){
        T2CONbits.TON = 1; // starts the timer!
    }
    
}