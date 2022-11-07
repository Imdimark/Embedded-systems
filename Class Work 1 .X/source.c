#include "header.h"

/*Timer Functions*/
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

void tmr_setup_period(int timer, int ms){
    if (timer == 1){
        T1CONbits.TON = 0;
        TMR1 = 0; // reset the current value;
        int tckps, pr;
        choose_prescaler(ms, &tckps, &pr);
        T1CONbits.TCKPS = tckps;
        PR1 = pr;
        T1CONbits.TON = 1;
        return;  
    }
    else if (timer == 2){
        T2CONbits.TON = 0;
        TMR2 = 0; // reset the current value;
        int tckps, pr;
        choose_prescaler(ms, &tckps, &pr);
        T2CONbits.TCKPS = tckps;
        PR2 = pr;
        T2CONbits.TON = 1;
        return;
    }
    else if (timer == 3){
        T3CONbits.TON = 0;
        TMR3 = 0; // reset the current value;
        int tckps, pr;
        choose_prescaler(ms, &tckps, &pr);
        T3CONbits.TCKPS = tckps;
        PR3 = pr;
        T3CONbits.TON = 1;
        return;
    }
    
}

void tmr_wait_period(int timer){
    if (timer == 1){
        while ( IFS0bits .T1IF == 0){}
        // I will exit the above loop only when the timer 1 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T1IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
    else if (timer == 2){
        while ( IFS0bits .T2IF == 0){}
        // I will exit the above loop only when the timer 2 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T2IF = 0; // set to zero to be able to recognize the next time the timer has expired
        return;
    }
    else if (timer == 3){
        while ( IFS0bits .T3IF == 0){}
        // I will exit the above loop only when the timer 3 peripheral has expired and it has set the T1IF flag to one
        IFS0bits .T3IF = 0; // set to zero to be able to recognize the next time the timer has expired
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
    return;
}

/*SPI Functions*/
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
    int i = 0;
    for(i = 0; i < 16; i++) {
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


/*UART Functions*/
void write_first_row(int count){
    // Char read from UART and sent to LCD
    while (U2STAbits.URXDA == 1){
        char c  = U2RXREG;
        count ++;
        
        // FIRST CHECK PT. 2/3/4 ASS
        if (c == '\r' || c == '\n' || count == 16){
            spi_clear_first_row();
            spi_move_cursor(1, 1);
        }
        // WRITE CHAR TO FIRST ROW
        spi_put_char(c);
        spi_move_cursor(1, count + 1);
    }
    if (U2STAbits.OERR ==1){
        while (U2STAbits.URXDA ==1){
            char c  = U2RXREG;
            count ++;
            // FIRST CHECK PT. 2/3/4 ASS
            if (c == '\r' || c == '\n' || count == 16){
                spi_clear_first_row();
                spi_move_cursor(1, 1);
            }
            // WRITE CHAR TO FIRST ROW
            spi_put_char(c);
            spi_move_cursor(1, count + 1);
        }
        U2STAbits.OERR ==0;
    }
}
void write_second_row(int count, char buff[]){
    // WRITE COUNT TO SECOND ROW PT. 5 ASS
    spi_clear_second_row();
    spi_move_cursor(2, 1);
    sprintf(buff, "Char Recv: %d", count);
    spi_put_string(buff);    
}