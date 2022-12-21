/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 * Readme: 
 *  Please note that the code is designed to receive data in that from  $MCREF,400* from the UART,
 *  if the user should only send the RPM from range to 0 to 1000 RPM, other wise it will be neglected. 
 *  Also, make sure not to flush the UART with characters as the buffer size is set for exactly the indicated
 *  message length with a safety factor of an one or two extra characters.
 */
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
#include "parser.h"

//Global Variables Initialization
typedef struct {
    char *buffer;
    int readIndex;
    int writeIndex;
}CircularBuffer;
typedef struct {
    int n;
    int N;
    void (*f)(void *);
    void* params;
}heartbeat;
typedef struct {		
    int speed;
    int error_flag;
}sp;

volatile CircularBuffer RXcircularBuffer;
volatile CircularBuffer TXcircularBuffer;
static char TX_buffer_size[20];
static char RX_buffer_size[8];
parser_state pstate;


//Functions Interrupt & Service Routine
void write_buffer(volatile CircularBuffer* cb, char value, int size){
    cb ->buffer[ cb -> writeIndex] = value;
    cb -> writeIndex = (cb -> writeIndex + 1) % size;
    if(cb -> readIndex == cb -> writeIndex){
        cb ->readIndex++;
    }
}
int read_buffer(volatile CircularBuffer *cb, char *value, int size){
    if(cb->readIndex == cb->writeIndex)
        return 0;
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if(cb->readIndex == size)
        cb->readIndex = 0;
    return 1;
}
int avl_bytes_cb(volatile CircularBuffer* cb, int size){
    if(cb -> readIndex <= cb -> writeIndex) {
        return cb->writeIndex - cb -> readIndex;
    }else{
        return size - cb ->readIndex + cb->writeIndex;
    }
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(){
    //ISR of UART2
    IFS1bits.U2RXIF = 0;
    char val = U2RXREG;
    write_buffer(&RXcircularBuffer, val, sizeof(RX_buffer_size));
}
void __attribute__((__interrupt__,__auto_psv__)) _U2TXInterrupt(){
    IFS1bits.U2TXIF = 0;
    char packetchar;
    while(U2STAbits.UTXBF == 0) {
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 1){
            U2TXREG = packetchar;
        }
        else{
            break;
        }
    }
}

void RX_UART(void* param) { 
    sp* structure = (sp*) param;
    int avl = avl_bytes_cb(&RXcircularBuffer, sizeof(RX_buffer_size)); 
    int count = 0;
    int RPMdesired = 0;
    while (count <avl ){
        char byte;
        IEC1bits.U2RXIE=0;
        read_buffer(&RXcircularBuffer, &byte, sizeof(RX_buffer_size));
        IEC1bits.U2RXIE=1;
        int ret = parse_byte(&pstate, byte);
        if (ret == NEW_MESSAGE){
            if(strcmp(pstate.msg_type, "MCREF" ) == 0){
                if (extract_number(pstate.msg_payload, &RPMdesired)!=-1){
                    if (RPMdesired>=0 && RPMdesired<=1000){
                            structure -> speed = RPMdesired;
                            structure ->error_flag =0;
                        }
                    }
                }
            else{
                structure ->error_flag =1;
            }
            }
        count++;
        
        }
}
void Set_PWM(void* param) {
	sp* structure = (sp*) param;
	double received_speed = structure -> speed;
    double duty_cycle = 0.25;
    duty_cycle = 0.05 + (received_speed/1000);
	PDC2 = duty_cycle * 2 * PTPER;
}
void Blink_Led(void* param) {
    sp* structure = (sp*) param;
    if(structure ->error_flag ==1 || U2STAbits.OERR == 1){
        if(U2STAbits.OERR == 1){
            U2STAbits.OERR = 0;
        }
        LATBbits.LATB0=0;
    }
    else{
        LATBbits.LATB0 = !LATBbits.LATB0;
    }
}
void TX_UART(void* param) {
    char packet[18];
    
    ADCON1bits.SAMP = 1; //start sampling
    while(ADCON1bits.DONE == 0); //wait until conversion is done
    ADCON1bits.DONE = 0; //just for double checking
    int potBits = ADCBUF0; // extract the value from the buffer
    double current = (potBits) * (50.0) / (1024)  -30.0; //mapping range from 0:1024 bits to -30:20 amps
    if(abs(current)>15.0){
        LATBbits.LATB1 = 1;
    }
    else if (abs(current)<=15.0){
        LATBbits.LATB1 = 0;
    }
    
    int tempBits = ADCBUF1; // extract the value from the buffer1
    double temperature = ((tempBits * 5.0/1024.0)-0.75)*100.0+25; //changing from bits range to volts range and then to degree Celsius range
    
    sprintf (packet,"$MCFBK,%.2f,%.2f*", current, temperature); //create the packet to be sent
    for(int i = 0; packet[i] != '\0'; i++) {
            write_buffer(&TXcircularBuffer, packet[i], sizeof(TX_buffer_size)); //store packet in TXcircularBuffer
        }
    IEC1bits.U2TXIE=0;
    char packetchar;
    while(U2STAbits.UTXBF==0){
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 1){
            U2TXREG = packetchar;
            }
            else{
                break;
            }
    }
    IEC1bits.U2TXIE=1;
}

void scheduler(heartbeat schedInfo[]) {
    int i;
    for (i = 0; i < MAX_TASKS; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            schedInfo[i].f(schedInfo[i].params);
            schedInfo[i].n = 0;
        }
    }
}

void setup(){
    /*PINS setup*/
    TRISBbits.TRISB0 = 0; //D3
    TRISBbits.TRISB1 = 0; //D4
    /*UART setup*/
    TXcircularBuffer.buffer= TX_buffer_size;
    RXcircularBuffer.buffer= RX_buffer_size;
    UART_config();
    /*ADC setup*/
    adc_config();
    /*PWM setup*/
    pwm_config();
    /*Parser setup*/
    pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    /*Wait for 1 second*/
    tmr_wait_ms(TIMER2, 1000);
}
int main(void) {
    setup();
    /*scheduler setup*/
    heartbeat schedInfo[MAX_TASKS];
    sp structure;
    //Setting the time of each task
    schedInfo[0].n = 0; 
    schedInfo[0].N = 1;
    schedInfo[0].f = RX_UART; 
    schedInfo[0].params = (void*)(&structure);
    schedInfo[1].n = 0;
    schedInfo[1].N = 30;
    schedInfo[1].f = Set_PWM;
    schedInfo[1].params = (void*)(&structure); 
    schedInfo[2].n = 0;
    schedInfo[2].N = 40;
    schedInfo[2].f = Blink_Led;
    schedInfo[2].params = (void*)(&structure); 
    schedInfo[3].n = 0;
    schedInfo[3].N = 40;
    schedInfo[3].f = TX_UART;
    schedInfo[3].params = NULL;
    structure.speed = 0;
    structure.error_flag = 0;
    
    tmr_setup_period(TIMER1, 5);
    while (1) {
        scheduler(schedInfo);
        tmr_wait_period(TIMER1);
    }
    return 0;
}

