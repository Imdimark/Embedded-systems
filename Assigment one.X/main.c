/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 */

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

//Libraries included
#include "header.h"

//Global Variables Initialization
int written = 0; //Initialization of the integer used in counting the characters received
int count = 0; //Initialization of the integer used in setting the cursor position
typedef struct {
    char buffer[BUFFER_SIZE];
    int readIndex;
    int writeIndex;
}CircularBuffer;
volatile CircularBuffer cb;

//Interrupt Service Routine & Functions
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt(){
    //ISR for S5 button
    IFS0bits.INT0IF = 0; //setting the flag down of Interrupt 0 
    
    //If S5 is pressed, send the current number of chars received to UART2
    char str2[]=" ";
    sprintf(str2, "%d", written);
    for (int i=0;str2[i] != '\0';i++){
        U2TXREG= str2[i];
    }
    
    tmr_setup_period(TIMER2,100); //disable the interrupt for some time for bouncing effect canceling
    IEC0bits.INT0IE = 0; //disable interrupt on S5
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    //ISR for S6 button
    IFS1bits.INT1IF = 0; //setting the flag down of Interrupt 1 down
    
    //S6 is pressed, clear the first row and reset the characters received counter
    written = 0;
    count = 0;
    spi_clear_first_row();
    spi_clear_second_row();
    spi_move_cursor(0, 0);
    
    tmr_setup_period(TIMER2,100);//disable the interrupt for some time for bouncing effect canceling
    IEC1bits.INT1IE = 0; //disable interrupt on S6
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(){
    //Used by s5 & s6 ISRs'
    T2CONbits.TON = 0; //stop the TIMMER2 because its not needed again
    IFS0bits.T2IF = 0; //setting the flag down of timer Interrupt 
    
    IFS0bits.INT0IF = 0; //setting the flag down of Interrupt 0
    IFS1bits.INT1IF = 0; //setting the flag down of Interrupt 1
    IEC0bits.INT0IE = 1; //enable interrupt on s5 again
    IEC1bits.INT1IE = 1; //enable interrupt on s6 again
}

void write_buffer( volatile CircularBuffer *cb, char value){
    //Function called by UART2 ISR to fill the buffer
    cb->buffer[cb->writeIndex] = value;
    cb-> writeIndex++;
    if(cb->writeIndex == BUFFER_SIZE)
        cb->writeIndex = 0;
}

int read_buffer(volatile CircularBuffer *cb, char *value){
    //Function called to read the buffer in the main
    if(cb->readIndex == cb->writeIndex)
        return 0;
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if(cb->readIndex == BUFFER_SIZE)
        cb->readIndex = 0;
    return 1;
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(){
    //ISR of UART2
    IFS1bits.U2RXIF = 0;
    char val = U2RXREG;
    write_buffer(&cb, val);
}


int main(void) {
    //Local Variables Initialization
    cb.writeIndex=0; //Initialization of the index used to write
    cb.readIndex =0; //Initialization of the index used to read
    char value; //Initialization of the character used in reading and writing 
    int read; //Initialization of the integer used in checking if there is data in the buffer 
    
    //Interrupt SETUP
    IEC0bits.INT0IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S5 button
    IEC1bits.INT1IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S6 button
    IEC1bits.U2RXIE = 1; //Interrupt flag HIGH; to enable interrupt on UART2
    
    //SPI SETUP
    /*
     * Setting the SPI1 module to operate in 8-bit Master mode,
     * the prescalers are set in order to not overcome the serial clock frequency (1 MHz)
     * FCY = (7,3728 MHz /4 ) = 1,8432 MHz
     * FSCK = FCY/(primary prescaler * secondary prescaler) = FCY/(1 * 2) = 0,9216 MHz
    */
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    //UART SETUP
    U2BRG = 11; //(7372800/4) / (16/9600)-1 at 9600 is the baud rate
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX
    
    tmr_wait_ms(TIMER3, 1000); //To allow the LCD to begin normal operation
    tmr_setup_period(TIMER1, 10); //Setup TIMER1 to run at 100 Hz
    while(1){
        tmr_wait_ms(TIMER3, 7); //Simulate an algorithm that needs 7 ms for its execution
        
        read = read_buffer(&cb, &value);
        if (read == 1){
            //There is data in the buffer
            if (value == '\r' || value == '\n'||count==16){
                //Whenever a CR ?\r? or LF ?\n? character is received, clear the first row
                count = 0;
                spi_clear_first_row();
            }
            spi_move_cursor(0, count);
            spi_put_char(value);
            count++;
            written++;
        }
        write_second_row(written);
        
        tmr_wait_period(TIMER1);
    }
    return (0);
}
