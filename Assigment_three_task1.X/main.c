/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 * Readme: The program has been set to work with a baud rate of 4800. The code
 * starts in timeout mode (visible on led D4 blinking). In order to reactivate and enter in 
 * control mode is possible to send the reference command. To Set in safe mode pressing the  
 * S5 button and for exit from it send the ENA message. The system, without receiving message  
 * will enter again in timeout mode.
 * 
 * PWM2L(RE2_2) and PWM2H(RE3_2) must be connected with left motor H-Bridge controller
 * PWM3L(RE4_2) and PWM3H (RE3_2) must be connected with right motor H-Bridge controller
 * 
 * Commands to send:
 * $HLREF,omega,speed*
 * $HLENA*
 * Commands to receive:
 * $MCFBK,n1,n2,state*
 * $MCALE,n1,n2*
 * $MCTEM,temp*
 * $MCACK,msg_type,value*
 * 
 * Four tasks have been created with different frequency:
 * Task1 at 10 HZ 
 * Task2 at 5 HZ
 * Task3 at 2 HZ
 * Task4 at 1 HZ
 *  
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
    double n1;
    double n2;
    double N1;
    double N2;
    double v;
    double w;
    int saturation_flag;
    int timeout_flag;
    int safemode_s5_flag;
    int lcd_toogle_s6_flag;
    float Temp_arr[TEMP_ARR_SIZE];
    int Temp_arr_index;
    double temp_avg;
    int overflow_flag;
}data;
data structure;
volatile CircularBuffer RXcircularBuffer;
volatile CircularBuffer TXcircularBuffer;
/*
 * The choice of TX_buffer_size:
 * -Task 1: TX is max 15 chars "$MCACK,ENA,1*\0", 15 chars is 120 bits at 10 Hz
 *  That's 1200 bits per second, as the Baud rate is 4800 bits per second. Therfore,
 *  nothing will be left in the buffer. so max TX_buffer_size is 15 now. 
 * -Task 2: TX is max 24 chars "$MCFBK,%-50.0,-50.0,1*\0", 24 chars is 192 bits at 5 Hz
 *  That's 960 bits per second, as the Baud rate is 4800 bits per second. Therfore,
 *  nothing will be left in the buffer. so max TX_buffer_size is 24 now.
 * -Task 3: TX is max 38 chars "$MCALE,-500.0,-500.0*\0$MCTEM,140.0*\0", 38 chars is 392 bits at 1 Hz
 *  That's 392 bits per second, as the Baud rate is 4800 bits per second. Therfore,
 *  nothing will be left in the buffer. so max TX_buffer_size is 38 now.
 *  However, Task 3 writes MCALE message first and then MCTEM message, and as the baud rate is fast enough
 *  to read whats inside the buffer so the size of the buffer should be the number of chars of the longer
 *  message which is 24 and for safety it is set as 30.
 */
static char TX_buffer_size[30];
/*
 * The choice of RX_buffer_size:
 * -Task 1: RX is max 15 chars "$HLREF,-500.0,-500.0*\0$HLENA*\0", 32 chars is 256 bits at 10 Hz
 *  That's 2560 bits per second, as the Baud rate is 4800 bits per second. Therfore,
 *  nothing will be left in the buffer. so max TX_buffer_size can be 32 now. However, the user
 *  should not send both HLREF and HLENA message at the same time and therfore the buffer size 
 *  should be chosen for the longer message which in this case is 23 and for safety it is set as 30.
 */
static char RX_buffer_size[30];
parser_state pstate;


//Functions Interrupt & Service Routine
void write_buffer(volatile CircularBuffer* cb, char value, int size){
    cb ->buffer[ cb -> writeIndex] = value;
    cb -> writeIndex = (cb -> writeIndex + 1) % size;
    if(cb -> readIndex == cb -> writeIndex){
        cb ->readIndex++;
    }
}
int read_buffer(volatile CircularBuffer* cb, char* value, int size) {
    if (cb -> readIndex != cb -> writeIndex) {
        *value = cb -> buffer[cb -> readIndex];
        cb -> readIndex = (cb->readIndex + 1) %size;
        return 0;
    }
    else{
        return -1;
    }
}
int avl_bytes_cb(volatile CircularBuffer* cb, int size){
    if(cb -> readIndex <= cb -> writeIndex) {
        return cb->writeIndex - cb -> readIndex;
    }
    else{
        return size - cb ->readIndex + cb->writeIndex;
    }
}

//Interrupt Service Routine & Functions
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(){
    //ISR of UART2
    IFS1bits.U2RXIF = 0;
    while (U2STAbits.URXDA == 1){
        write_buffer(&RXcircularBuffer, U2RXREG, sizeof(RX_buffer_size));
    }
    if(U2STAbits.OERR == 1){
        U2STAbits.OERR = 0;
        structure.overflow_flag = 1;
    }
}
void __attribute__((__interrupt__,__auto_psv__)) _U2TXInterrupt(){
    IFS1bits.U2TXIF = 0;
    char packetchar;
    while(U2STAbits.UTXBF == 0) {
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 0){
            U2TXREG = packetchar;
        }
        else{
            
            break;
        }
        
    }
}
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt(){
    //ISR for S5 button
    IFS0bits.INT0IF = 0; //setting the flag down of Interrupt 0 
    
    //Enable safe_mode
    PDC2 = PTPER;
    PDC3 = PTPER;
    structure.safemode_s5_flag = 1;
    structure.saturation_flag = 0;
    structure.n1=structure.N1=structure.n2=structure.N2= structure.v=structure.w = structure.timeout_flag =0;
    
    tmr_setup_period(TIMER2,100); //disable the interrupt for some time for bouncing effect canceling
    IEC0bits.INT0IE = 0; //disable interrupt on S5
    IEC0bits.T2IE = 1; //enable interrupt on timer2
}
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    //ISR for S6 button
    IFS1bits.INT1IF = 0; //setting the flag down of Interrupt 1 down
    
    //S6 is pressed, toggle LCD flag
    structure.lcd_toogle_s6_flag = !structure.lcd_toogle_s6_flag; 
    
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
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(){
    T1CONbits.TON = 0; //stop the TIMMER1 because its not needed again
    IFS0bits.T1IF = 0; //setting the flag down of timer Interrupt
    
    //set flag high.
    structure.timeout_flag = 1;
}


void UARTTX(char* packet) {
    for(int i = 0; packet[i] != '\0'; i++) {
        write_buffer(&TXcircularBuffer, packet[i], sizeof(TX_buffer_size)); //store packet in TXcircularBuffer
    }
    IEC1bits.U2TXIE=0;
    char packetchar;
    while(U2STAbits.UTXBF==0){
        if(read_buffer(&TXcircularBuffer, &packetchar, sizeof(TX_buffer_size)) == 0){
            U2TXREG = packetchar;
        }
        else{
        
            break;
        }

    }
    IEC1bits.U2TXIE=1;
}
float get_average(float new_element) {
    structure.Temp_arr[structure.Temp_arr_index] = new_element;
    structure.Temp_arr_index++;
    if(structure.Temp_arr_index < TEMP_ARR_SIZE){
        return -1;
    }
    else {
        structure.Temp_arr_index = 0;
        float sum = 0;
        for (int i = 0; i < TEMP_ARR_SIZE; i++) {
            sum += structure.Temp_arr[i];
        }
        return sum / TEMP_ARR_SIZE;
    }
}

void task1(void* param) { 
    data* structure = (data*) param;
    IEC1bits.U2RXIE=0;
    int avl = avl_bytes_cb(&RXcircularBuffer, sizeof(RX_buffer_size));
    IEC1bits.U2RXIE=1;
    int count = 0;
    while (count <avl){
        char byte;
        IEC1bits.U2RXIE=0;
        read_buffer(&RXcircularBuffer, &byte, sizeof(RX_buffer_size));
        IEC1bits.U2RXIE=1;
        int ret = parse_byte(&pstate, byte);
        if (ret == NEW_MESSAGE){
            if(strcmp(pstate.msg_type, "HLREF") == 0  && structure->safemode_s5_flag ==0){
                if (extract_numbers(pstate.msg_payload, &structure->w, &structure->v)==1){
                    structure->timeout_flag = 0; //reset timeout_flag
                    tmr_setup_period(TIMER1,5000);
                    calculate_wheel_speeds(structure->v,structure->w,&structure->N1, &structure->N2);
                    if (structure->N1 < -limit || structure->N1 > limit || structure->N2 < -limit || structure->N2 > limit) {
                        //Saturation occurred 
                        IEC0bits.INT0IE = 0;
                        structure->saturation_flag=1;
                        IEC0bits.INT0IE = 1;
                        structure->n1 = (structure->N1 > limit) ? limit : ((structure->N1 < -limit) ? -limit : structure->N1);
                        structure->n2 = (structure->N2 > limit) ? limit : ((structure->N2 < -limit) ? -limit : structure->N2);
                    }
                    else{
                        IEC0bits.INT0IE = 0;
                        structure->saturation_flag=0;
                        IEC0bits.INT0IE = 1;
                        structure->n1=structure->N1;
                        structure->n2=structure->N2;
                    }
                }
            }
            else if(strcmp(pstate.msg_type, "HLENA" ) == 0){
                //Disable safemode
                IEC0bits.INT0IE = 0;
                structure->safemode_s5_flag = 0;
                IEC0bits.INT0IE = 1;
                structure->n1=structure->N1=structure->n2=structure->N2 = 0;
                //Send ACK of ENA
                char packet[] = "$MCACK,ENA,1*"; 
                UARTTX(packet);
            }
        }
        count++;
    }
    //PWM
    double duty_cycle_n1= map(structure->n1,-60,60,0,1);
    double duty_cycle_n2= map(structure->n2,-60,60,0,1);
    PDC2 = duty_cycle_n1 * 2 * PTPER;
    PDC3 = duty_cycle_n2 * 2 * PTPER;
    
    
    //TEMP part here
    ADCON1bits.SAMP = 1; //start sampling
    while(ADCON1bits.DONE == 0); //wait until conversion is done
    ADCON1bits.DONE = 0; //just for double checking
    int tempBits = ADCBUF0; // extract the value from the buffer 
    float temperature;
    temperature = ((tempBits * 5.0/1024.0)-0.75)*100.0+25.0;
    float avg = get_average(temperature);
    if (avg!=-1){
        structure ->temp_avg = avg;
    }
    
     
    //LCD
    char status = 'C';
    if (structure ->safemode_s5_flag ==1){
        status = 'H';
    }
    else if (structure ->safemode_s5_flag ==0 && structure->timeout_flag == 1){
        status = 'T';
    }
    else{
        status = 'C';
    }
    char row[17];
    sprintf(row, "STATUS: %c",status);
    spi_clear_first_row();
    spi_move_cursor(FIRST_ROW,0);
    spi_put_string(row);
    if (structure ->lcd_toogle_s6_flag == 0){
            sprintf(row, "R:%.1f,%.1f",structure->n1,structure->n2);
            spi_clear_second_row();
            spi_move_cursor(SECOND_ROW,0);
            spi_put_string(row);
    }
    else{
        char S [30]; // As its not knowen how long the the user message can be
        sprintf(S, "%.1f;%.1f",structure->w,structure->v);
        int i = strlen(S);
        if (i>=15){
            sprintf(row, "Too long print");
            spi_clear_second_row();
            spi_move_cursor(SECOND_ROW,0);
            spi_put_string(row);
        }
        else {
            sprintf(row, "S:%s",S); // the first letter is S not R just to differentiate between n1,n2
            spi_clear_second_row();
            spi_move_cursor(SECOND_ROW,0);
            spi_put_string(row);
        }
    }
    if(structure ->timeout_flag ==0){
        LATBbits.LATB1=0;
    }
    else if (structure ->timeout_flag ==1){
        LATBbits.LATB1 = !LATBbits.LATB1;
    }
}
void task2(void* param) {
    data* structure = (data*) param;
    int status = 0;
    if (structure ->timeout_flag ==1){
        IEC0bits.INT0IE = 0;
        structure->saturation_flag = 0;
        IEC0bits.INT0IE = 1;
        structure->n1=structure->N1=structure->n2=structure->N2 = structure->v=structure->w = 0;
    }
    
    if (structure ->safemode_s5_flag ==1){
        status = 2;
    }
    else if (structure ->safemode_s5_flag ==0 && structure->timeout_flag == 1){
        status = 1;
    }
    else{
        status = 0;
    }
    char packet[25];
    sprintf (packet,"$MCFBK,%.1f,%.1f,%d*", structure->n1, structure->n2,status);
    UARTTX(packet);
}
void task3(void* param) {
    data* structure = (data*) param;
    if(structure->overflow_flag == 1){
        LATBbits.LATB0 = 0;
    }
    else {
        LATBbits.LATB0 = !LATBbits.LATB0;
    }
    
}
void task4(void* param) {
    data* structure = (data*) param;
    char MCALE[25]; 
    char MCTEM[17];
    //Saturation
    if(structure ->saturation_flag == 1){
        sprintf(MCALE,"$MCALE,%.1f,%.1f*",structure->N1, structure->N2);
        UARTTX(MCALE);
    }
    // Temperature
    sprintf(MCTEM,"$MCTEM,%.1f*", structure->temp_avg);
    UARTTX(MCTEM);
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
    /*I/O SETUP*/
    TRISBbits.TRISB1 = 0; //D4
    TRISBbits.TRISB0 = 0; //D3
    /*Interrupt SETUP*/
    IEC0bits.INT0IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S5 button
    IEC1bits.INT1IE = 1; //Interrupt flag HIGH; to enable interrupt on pin S6 button
    IEC0bits.T1IE = 1; //enable interrupt on timer1
    /*UART setup*/
    TXcircularBuffer.buffer= TX_buffer_size;
    TXcircularBuffer.readIndex = 0;
    TXcircularBuffer.readIndex = 0;
    RXcircularBuffer.buffer= RX_buffer_size;
    RXcircularBuffer.readIndex = 0;
    RXcircularBuffer.writeIndex = 0;
    UART_config();
    
    /*LCD setup*/
    spi_config();
    /*ADC setup*/
    adc_config();
    /*PWM setup*/
    pwm_config();
    PDC2 = PTPER;
    PDC3 = PTPER;
    /*Parser setup*/
    pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    
    /*Wait for 1 second*/
    tmr_wait_ms(TIMER4, 1000);
}
int main(void) {
    setup();
    /*scheduler setup*/
    heartbeat schedInfo[MAX_TASKS];
    
    structure.N1 = 0;
    structure.N2 = 0;
    structure.n1 = 0;
    structure.n2 = 0;
    structure.v  = 0;
    structure.w  = 0;
    structure.saturation_flag  = 0;
    structure.timeout_flag  = 1;
    structure.safemode_s5_flag = 0;
    structure.lcd_toogle_s6_flag = 0;
    structure.Temp_arr_index = 0;
    structure.overflow_flag = 0;
    
    //Setting the time of each task
    schedInfo[0].n = 0; 
    schedInfo[0].N = 20; //10hz = 0.1 s = 0.005 * 20
    schedInfo[0].f = task1; 
    schedInfo[0].params = (void*)(&structure);
    schedInfo[1].n = 0;
    schedInfo[1].N = 40;
    schedInfo[1].f = task2;
    schedInfo[1].params = (void*)(&structure);
    schedInfo[2].n = 0;
    schedInfo[2].N = 100;
    schedInfo[2].f = task3; //PUT IN TASK1
    schedInfo[2].params = (void*)(&structure);    
    schedInfo[3].n = 0;
    schedInfo[3].N = 200;
    schedInfo[3].f = task4;
    schedInfo[3].params = (void*)(&structure);
    
    
    tmr_setup_period(TIMER3, 5);
    while (1) {
        scheduler(schedInfo);
        tmr_wait_period(TIMER3);
    }
    return 0;
}