/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 */
#include "header.h"

/*Timer Functions*/
void choose_prescaler(int ms, int* pr, int* tckps){
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
    if ( ticks <= 65535){
        *tckps = 3;
        *pr = ticks ;
        return;
    }
}

void tmr_setup_period(int timer, int ms){
    int tckps, pr;
    if (timer == 1){
        T1CONbits.TON = 0;
        TMR1 = 0; // reset the current value;
        choose_prescaler(ms, &pr, &tckps);
        T1CONbits.TCKPS = tckps;
        PR1 = pr;
        T1CONbits.TON = 1;
        return;  
    }
    else if (timer == 2){
        T2CONbits.TON = 0;
        TMR2 = 0; // reset the current value;
        choose_prescaler(ms, &pr, &tckps);
        T2CONbits.TCKPS = tckps;
        PR2 = pr;
        T2CONbits.TON = 1;
        return;
    }
    else if (timer == 3){
        T3CONbits.TON = 0;
        TMR3 = 0; // reset the current value;
        choose_prescaler(ms, &pr, &tckps);
        T3CONbits.TCKPS = tckps;
        PR3 = pr;
        T3CONbits.TON = 1;
        return;
    }
    else if (timer == 4){
        T4CONbits.TON = 0;
        TMR4 = 0; // reset the current value;
        choose_prescaler(ms, &pr, &tckps);
        T4CONbits.TCKPS = tckps;
        PR4 = pr;
        T4CONbits.TON = 1;
        return;
    }
}

void tmr_wait_period(int timer){
    if (timer == 1){
        while (IFS0bits.T1IF == 0){}
        // I will exit the above loop only when the timer 1 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T1IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
    else if (timer == 2){
        while (IFS0bits.T2IF == 0){}
        // I will exit the above loop only when the timer 2 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T2IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
    else if (timer == 3){
        while (IFS0bits.T3IF == 0){}
        // I will exit the above loop only when the timer 3 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T3IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
    else if (timer == 4){
        while (IFS1bits.T4IF == 0){}
        // I will exit the above loop only when the timer 4 peripheral has expired and it has set the T1IF flag to one
        IFS1bits.T4IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
}

void tmr_wait_ms(int timer, int ms) {
    if (timer == 1){
        int pr, tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR1 = pr;
        T1CONbits.TCKPS = tckps;
        T1CONbits.TCS = 0;
        T1CONbits.TGATE = 0;
            
        T1CONbits.TON = 0;
        IFS0bits.T1IF = 0;
        TMR1 = 0;
        T1CONbits.TON = 1;
        while (IFS0bits.T1IF == 0);
        IFS0bits.T1IF = 0;
        T1CONbits.TON = 0;
        return;
    }
    else if (timer == 2){
        int pr, tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR2 = pr;
        T2CONbits.TCKPS = tckps;
        T2CONbits.TCS = 0;
        T2CONbits.TGATE = 0;
            
        T2CONbits.TON = 0;
        IFS0bits.T2IF = 0;
        TMR2 = 0;
        T2CONbits.TON = 1;
        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 0;
        return;
        }
    else if (timer == 3){
        int pr, tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR3 = pr;
        T3CONbits.TCKPS = tckps;
        T3CONbits.TCS = 0;
        T3CONbits.TGATE = 0;
            
        T3CONbits.TON = 0;
        IFS0bits.T3IF = 0;
        TMR3 = 0;
        T3CONbits.TON = 1;
        while (IFS0bits.T3IF == 0);
        IFS0bits.T3IF = 0;
        T3CONbits.TON = 0;
        return;
    }
    else if (timer == 4){
        int pr, tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR4 = pr;
        T4CONbits.TCKPS = tckps;
        T4CONbits.TCS = 0;
        T4CONbits.TGATE = 0;
            
        T4CONbits.TON = 0;
        IFS1bits.T4IF = 0;
        TMR4 = 0;
        T4CONbits.TON = 1;
        while (IFS1bits.T4IF == 0);
        IFS1bits.T4IF = 0;
        T4CONbits.TON = 0;
        return;
    }
    return;
}

/*UART Functions*/
void UART_config(){
    
    /*
    * The choice of baud rate from both RX-TX point of view:
    *-Task 1: TX is max 47 chars "$MCACK,ENA,1*\0$HLREF,-500.0,-500.0*\0$HLENA*\0", 47 chars is 376 bits at 10 Hz
    *  That's 3760 bits per second. so min baud rate = 3760 bits per second.
    *-Task 2: TX is max 24 chars "$MCFBK,%-50.0,-50.0,1*\0", 24 chars is 192 bits at 5 Hz
    *  That's 960 bits per second, so still the min buffer size = 3760 bits per second.
    *-Task 3: TX is max 38 chars "$$MCALE,-500.0,-500.0*\0$MCTEM,140.0*\0", 38 chars is 392 bits at 1 Hz
    *  That's 392 bits per second, so still the min baud rate= 3760 bits per second.
    * 
    * However, 3760 bits per second is not a standard, so the least standard baud rate is 4800 bits per second.
    */
    U2BRG = 23; // For 4800 bits per second (Baud rate)
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX
    IEC1bits.U2RXIE =1; //interrupt enabled
}

/*ADC Functions*/
void adc_config() { //using sequential sampling
	//AUTOMATIC ALL, For temp chanel only
    ADCON3bits.ADCS = 8; //STEP1 
    ADCON1bits.ASAM = 1; //STEP2.2 bec its automatic
    ADCON1bits.SSRC = 7; //STEP3.2 bec its automatic conversion
    ADCON3bits.SAMC = 1; //STEP3.2
    ADCON2bits.CHPS = 0; //STEP4 //bec we only need to use chanel 0
    ADCHSbits.CH0SA = 3; // STEP5 3 for AN3 temp sens
    ADCHSbits.CH0NA = 0; // STEP5 AN3 wrt GND
    ADPCFG = 0xFFFF;     // STEP6 setup all as digital
    ADPCFGbits.PCFG3 = 0;// STEP6 setup AN3 as analog
    ADCON1bits.ADON = 1; //STEP9 turn on ADC
}

/*PWM Functions*/
void pwm_config(){
    PTCONbits.PTMOD = 0; // free running
    PTCONbits.PTCKPS = 0; //1:1 prescaler as 7372800/4=1843200/1000=1843.2-1=1842.2 can be in 15 bit for a frequency of 1 kHz
    PWMCON1bits.PEN2H = 1; // Enable PWM2H
    PWMCON1bits.PEN2L = 1; // Enable PWM2L
    PWMCON1bits.PEN3H = 1; // Enable PWM3H
    PWMCON1bits.PEN3L = 1; // Enable PWM3L
    DTCON1bits.DTA=6;      // Dead time 3ms
    DTCON1bits.DTAPS=0;    //Dead time prescaler
    PTPER = 9216; //Fcy/(4*50Hz)-1
    PTCONbits.PTEN = 1; //Enable PWM       
    // Tcy = 543 ns
}


/*SPI Functions*/

void spi_config(){
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
}

void spi_put_char(char c) {
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = c;
}

void spi_put_string(char* str) {
    int i = 0;
    for(i = 0; str[i] != '\0'; i++) {
        spi_put_char(str[i]);
    }
}

void spi_move_cursor(int row, int column) {
    switch (row) {
        case 0:
            spi_put_char(0x80 + column);
            return;
        case 1:
            spi_put_char(0xC0 + column);
            return;
    }
}

void spi_clear_first_row() {
    spi_move_cursor(FIRST_ROW, 0);
    for(int i = 0; i < 16; i++) {
        spi_put_char(' ');
    }
}

void spi_clear_second_row() {
    spi_move_cursor(SECOND_ROW, 0);
    int i = 0;
    for(i = 0; i < 16; i++) {
        spi_put_char(' ');
    }
}

/*FOR FINAL PROJECT*/
void calculate_wheel_speeds(double v, double w, double* wheel1_rpm, double* wheel2_rpm){
    const float pi = 3.14159265358979323846;
    const float r = 0.2; // wheel radius (in meters)
    const float d = 0.5; // distance between wheels (in meters)

    *wheel1_rpm = ((v + (d/2) * w) / (2 * pi * r))*60;
    *wheel2_rpm = ((v - (d/2) * w) / (2 * pi * r))*60;
}
double map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}