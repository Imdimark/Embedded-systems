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
int led=1;
int main(void) {
    //TRISDbits.TRISD0 = 1; // set the pin as input "button S6"
    TRISEbits.TRISE8 = 1; // set the pin as input "button S5"
    TRISBbits.TRISB0 = 0; // set the pin as output "led D3"

    //1,2)
    /*
    while(1){
    if(PORTEbits.RE8==0){
        LATBbits.LATB0 = 1; // set the pin high "led D3"
    }
    else{
        LATBbits.LATB0 = 0; // set the pin low "led D3" 
    }
    }*/
    
    //3)
    
    while(1){
        if(PORTEbits.RE8==0 && led==1){
            led=0;
            LATBbits.LATB0 = led; // set the pin high "led D3"
            while(PORTEbits.RE8==0){}
        }
        else if(PORTEbits.RE8==0 && led==0){
            led=1;
            LATBbits.LATB0 = led; // set the pin high "led D3"
            while(PORTEbits.RE8==0){}
        }
    }
    return 0;
}