/*
 * Authors: 
 *  Youssef Mohsen Mahmoud Attia 5171925
 *  Giovanni Di Marco            5014077
 *  Sinatra Gesualdo             5159684
 */
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#include <stdio.h>
#include <stdlib.h>
#include <xc.h> // include processor files - each processor file is guarded.  

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define FIRST_ROW 0
#define SECOND_ROW 1
#define BUFFER_SIZE 60


 
/*Timer Fucntions*/
void choose_prescaler(int ms, int* pr, int* tckps);
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);

/*SPI Functions*/
void spi_put_char(char c);
void spi_put_string(char* str);
void spi_move_cursor(int row, int column);
void spi_clear_first_row();
void spi_clear_second_row();

/*UART Functions*/
void write_second_row(int written);

#endif	/* XC_HEADER_TEMPLATE_H */

