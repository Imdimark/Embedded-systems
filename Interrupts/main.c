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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "xc.h"
void choose_prescaler(int ms, int *tckps, int *pr){
    long ticks = 1843.2*ms;
    if (ticks <= 65535){
        *tckps = 0;
        *pr = ticks;
    }
    ticks = ticks/8;
    if (ticks <= 65535){
        *tckps = 1;
        *pr = ticks;
    }
    ticks = ticks/8;
    if (ticks <= 65535){
        *tckps = 2;
        *pr = ticks;
    }
    ticks = ticks/4;
    if (ticks <= 65535){
        *tckps = 3;
        *pr = ticks;
    }  
}

void tmr1_setup_period(int ms){
    T1CONbits.TON = 0;
    TMR1 = 0;
    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr);
    T1CONbits.TCKPS = tckps;
    PR1 = pr;
    T1CONbits.TON = 1;
    return;
}

void tmr2_setup_period(int ms){
    T2CONbits.TON = 0;
    TMR2 = 0;
    int tckps, pr;
    choose_prescaler(ms, &tckps, &pr);
    T2CONbits.TCKPS = tckps;
    PR1 = pr;
    T2CONbits.TON = 1;
    return;
}


void tmr1_wait_period(){
    while(IFS0bits.T1IF==0){ //exit only when timer1 has expired
        IFS0bits.T1IF=0; //reset timer
        IFS0bits.T2IF=0; //resetta timer 2 per la seconda volta a fine 500, quindi 0--primo-->250--secondo-->500
    }
    
}

void __attribute__ ((__interrupt__, __auto_psv__ )) _T2Interrupt(){
    IFS0bits.T1IF=0; //entra, resetta il timer prima volta ciclo così riparte
    
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    LATBbits.LATB1=!LATBbits.LATB1;//toggleandwritethevaluetothepin 
    
}

int main(void) {
    IEC0bits.INT0IE = 1; //enable int0 interrupt
    
    IEC0bits.T2IE = 1; //enable timer2 interrupt, every time that IFS0bits.T2IF becomes 1
    
    TRISBbits.TRISB0=0;
    TRISBbits.TRISB1=0;
    tmr1_setup_period(500);
    tmr2_setup_period(250);    
    while(1){ //thisloopisexecutedonceevery500ms 
        LATBbits.LATB0=!LATBbits.LATB0;//toggleandwritethevaluetothepin 
        tmr1_wait_period();//waitwhatisneededforthenextloop 
    }
}
