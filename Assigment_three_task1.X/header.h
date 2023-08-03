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
#include <string.h>
#include "stdlib.h"
#include <xc.h> // include processor files - each processor file is guarded.  

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define FIRST_ROW 0
#define SECOND_ROW 1
#define MAX_TASKS 4
#define limit 50
#define TEMP_ARR_SIZE 10

 
/*Timer Fucntions*/
void choose_prescaler(int ms, int* pr, int* tckps);
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);

/*UART Functions*/
void UART_config();

/*ADC Functions*/
void adc_config();

/*PWM Functions*/
void pwm_config();

/*SPI Functions*/
void spi_config();
void spi_put_char(char c);
void spi_put_string(char* str);
void spi_move_cursor(int row, int column);
void spi_clear_first_row();
void spi_clear_second_row();


/*FOR FINAL PROJECT*/
void calculate_wheel_speeds(double v, double w, double* wheel1_rpm, double* wheel2_rpm);
double map(double x, double in_min, double in_max, double out_min, double out_max);
#endif	/* XC_HEADER_TEMPLATE_H */

