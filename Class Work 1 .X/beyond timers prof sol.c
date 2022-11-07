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
#include "header.h"

/*
 * 
 */
int main(void) {
    TRISEbits.TRISE8 = 1; // set the pin as input "button S5"
    TRISBbits.TRISB0 = 0; // set the pin as output "led D3"
    
    int periodsCounter = 0;
    int prevButtonStatus = PORTEbits.RE8;
    int numberPulses = 0;
    int currentButtonStatus;
    int ButtonPressedPeriods = 0;
    
    tmr_setup_period(TIMER1, 100);
    
    
    while(1){
        periodsCounter++;
        if (periodsCounter == 10){
            periodsCounter = 0;
        }
        currentButtonStatus = PORTEbits.RE8;
        if(currentButtonStatus == 0 &&  prevButtonStatus == 1){ //button got pressed and not pressed before
            numberPulses++;
            if(numberPulses==4){
                numberPulses=1;
            }
            else if (currentButtonStatus ==0 && prevButtonStatus ==0){ //button is being pressed
                ButtonPressedPeriods++;
                if (ButtonPressedPeriods>30){ //30 periods is 3 sec of bottn pressing
                    numberPulses = 0;
                }
            }
            else if (currentButtonStatus ==1 && prevButtonStatus ==0){ // button is presseing is relased
                if (ButtonPressedPeriods>30){ //30 periods is 3 sec of bottn pressing
                    numberPulses = 0;
                }
                ButtonPressedPeriods=0;
            }
        }
        prevButtonStatus = currentButtonStatus;
        if (periodsCounter%2 == 1){ //it is odd
            if (periodsCounter< 2*numberPulses){
                LATBbits.LATB0 = 1;
            }
            else {
                LATBbits.LATB0 = 0;
            }
        }
        else{
            LATBbits.LATB0 = 0;
        }
        tmr_wait_period(TIMER1);
    }
    
    return (0);
}

