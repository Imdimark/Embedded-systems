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
#define delay 25
#define HIGH 1
#define LOW 0
void choose_prescaler(int ms, int* tckps, int* pr);
void tmr1_setup_period(int ms);
void tmr1_wait_period();

int main(void) {
    //TRISDbits.TRISD0 = 1; // set the pin as input "button S6"
    //TRISEbits.TRISE8 = 1; // set the pin as input "button S5"
    TRISBbits.TRISB0 = 0; // set the pin as output "led D3"
    tmr1_setup_period(delay);
    while(1){
        LATBbits.LATB0 = HIGH;
        tmr1_wait_period();
        LATBbits.LATB0 = LOW;
        tmr1_wait_period();
    }
    return 0;
}

void choose_prescaler(int ms, int* tckps, int* pr){
    //Fcy = 1843200 Hz ?> 1843,2 clock ticks in 1 ms
    long ticks = 1843.2*ms; // there can be an approximation
    if ( ticks <= 65535){ // if ticks is > 65535 it cannot be put in PR1 (only 16 bits )
        *tckps = 0;
        *pr = ticks ;
        return;
    }
    ticks = ticks / 8; // prescaler 1:8;
    if ( ticks <= 65535) {
        *tckps = 1;
        *pr = ticks ;
        return;
    }
    ticks = ticks / 8; // prescaler 1:64;
    if ( ticks <= 65535) {
        *tckps = 2;
        *pr = ticks ;
        return;
    }
    ticks = ticks / 4; // prescaler 1:256;
    *tckps = 3;
    *pr = ticks ;
    return;
}

void tmr1_setup_period(int ms){
    T1CONbits.TON = 0;
    TMR1 = 0; // reset the current value;
    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr);
    T1CONbits.TCKPS = tckps;
    PR1 = pr;
    T1CONbits.TON = 1;
    return;
}

void tmr1_wait_period() {
    while ( IFS0bits .T1IF == 0){}
    // I will exit the above loop only when the timer 1 peripheral has expired and it has set the T1IF flag to one
    IFS0bits .T1IF = 0; // set to zero to be able to recognize the next time the timer has expired
}
