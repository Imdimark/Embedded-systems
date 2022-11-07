//FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
//FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
//FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)
//FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)
//FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

//Libraries included
#include "header.h"

//Initialization global variables
int count = 0;
char buff [15] = "";

/*ISRs'*/
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt(){
    //S5 ISR
    IFS0bits.INT0IF = 0; //setting the flag down of int0 

    //S5 is pressed, send the current number of chars received to UART2
    U2RXREG = count;
    
    tmr_setup_period(TIMER2,100); //disable the interrupt for some time for bouncing effect canceling
    IEC0bits.INT0IE = 0; //disable interrupt on S5
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    //S6 ISR
    IFS1bits.INT1IF = 0; //setting the flag down of int1 down

    //S6 is pressed, clear the first row and reset the characters received counter
    count = 0;
    spi_clear_first_row();
    spi_move_cursor(1, 1);
    
    tmr_setup_period(TIMER2,100);//disable the interrupt for some time for bouncing effect canceling
    IEC1bits.INT1IE = 0; //disable interrupt on S6
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(){
    //used by s5 & s6 ISRs'
    T2CONbits.TON = 0; //stop the TIMMER2 because its not needed again
    IFS0bits.T2IF = 0; //setting the flag down of timmer int 
    
    IFS0bits.INT0IF = 0; //setting the flag down of int0
    IFS1bits.INT1IF = 0; //setting the flag down of int1
    IEC0bits.INT0IE = 1; //enable interrupt on s5 again
    IEC1bits.INT1IE = 1; //enable interrupt on s6 again
}

int main(void) {
    IEC0bits.INT0IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S5 button
    IEC1bits.INT1IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S6 button
    tmr_setup_period(TIMER1, 10);
    while(1){
        tmr_wait_ms(TIMER3, 7); //This is the 7 ms
        // Initialize to first position 
        // LCD and counter every cycle
        spi_move_cursor(1, 1);
        count = 0;
        
        // FIRST ROW PT. 2/3/4 ASS
        write_first_row(count);
        
        // SECOND ROW PT. 5 ASS
        write_second_row(count, buff);
        tmr_wait_period(TIMER1);
    }
    return (0);
}