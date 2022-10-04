/*
 * File:   main.c
 * Author: Aldo
 *

------------pin a disposizione------------------
 * switch s5 = pin 17 --> RE8
 * led d3 = pin 2 --> RB0
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




int led = 0;
void SetOnLed();
void SetOffLed();



void SetOnLed() { //It is also the first point
    LATDbits.LATD3 = 1;
    
}
void SetOffLed() {
    LATDbits.LATD3 = 0;
    
    
}


char UART_get_char()   
{

    
    while(!U2RXREG);  // hold the program till RX buffer is free
    
    return U2RXREG; //receive the value and send it to main function
}
//_____________End of function________________//
int main(void) {
    
    TRISBbits.TRISB0 = 0; //OUTPUT
    TRISEbits.TRISE8 = 1; // INPUT
   
    
    while(1){
  
        int pressed = PORTEbits.RE8;
        SetOffLed();
        if(pressed == 0 && led == 1){
        SetOffLed();
        led = 1;
    }
        else if(pressed == 0 && led == 0){
        SetOnLed();
        led = 0;
    }
        
        
    }
    
    
    
    
    
    
    
    return 0;} 