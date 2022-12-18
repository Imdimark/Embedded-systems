
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
#include <p30F4011.h>
#include <stdio.h>
#include <string.h>
#include "header.h"

#define MAX_TASKS 5

typedef struct {
    int n;
    int N;
    void (*f)(void *);
    void* params;

} heartbeat;

typedef struct {		
    double speed;
	double current;
	double temperature;
} ls;

typedef struct {
    char buffer[BUFFER_SIZE];
    int readIndex;
    int writeIndex;
}CircularBuffer;
volatile CircularBuffer cb;

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
char string[] = "This is a very long string";

void read_spe_uart(void* param) {  
    ls* structure = (ls*) param;
    int read;
    char value;
    char rpm[]="";
    char str[]="";
    int i=0;
    double rpmdouble=0;
    read = read_buffer(&cb, &value);
    if (read == 1){ //should hv been a while loop? YES, while read 1 everything in the circular buffer end
        str[i]=value;
        U2TXREG= str[i];
        i++;
        read = read_buffer(&cb, &value);
    }
    for (int j=0;str[j+6]!='\0';j++){	// while items in the circular buffer -> val = parse(&ps,item of cb) (val can be NEW_MESSAGE or NO_MESSAGE may useful for initialize the variable again)				
        rpm[j]=str[j+6];		// Process it and classify info!!
        U2TXREG= rpm[j];		//  if(ps->state == STATE_PAYLOAD) -> rpm[ps->index_payload] = ps -> msg_payload[ps->index_payload ] something like this...
	    				// then we can use fscanf for retrieve number from string like -> fscanf(rpm,"%f",ls.speed) end
    }					// if(val == NEW_MESSAGE) -> rpm[] = "" (? i do not know)
    
    structure -> speed = rpmdouble;
    //read from uart --> circular buffer --> parser -->
    
}

void set_voltage_DC(void* param) {
	ls* structure = (ls*) param;
	double received_speed = structure -> speed; //-------------------> here we store the received velocity
    double duty_cycle = 0;
	if (received_speed> 999.5) {
		duty_cycle = 1;
	}
	else if(0 < received_speed <= 999.5){
		duty_cycle = 0.05 + (received_speed/1000); //0.05 is the minimum to avoid distorsion (See dead time for example
	}
	else{
		duty_cycle = 0;
	}

	//duty_cycle = 0.05*(received_speed / 1023.0) + 0.05;
   
	PDC2 = duty_cycle; // * 2 * PTPER;
    
    /*char str2[]=" ";
    sprintf(str2, "%lf", received_speed);
    for (int i=0;str2[i] != '\0';i++){
        U2TXREG= str2[i];
    }*/
  
}

void blink_led(void* param) { //this point is finished
    LATBbits.LATB0 = !LATBbits.LATB0;
}


void read_potentiometer(void* param) {
	ls* structure = (ls*) param;
	char str[17]; //------>is 14 enough?
	while (1) // repeat continuously
	{
		ADCON1bits.SAMP = 1; // start sampling ...
		tmr_wait_ms(TIMER2,2); // for 100 mS
		ADCON1bits.SAMP = 0; // start Converting

		while (!ADCON1bits.DONE); // conversion done?
			int ADCValue_pot = ADCBUF0; // yes then get ADC value
			int ADCValu_temp = ADCBUF1; // yes then get ADC value
			
			double voltage_pot = ADCValue_pot/1024.0 * 5.0;
			double voltage_temp = ADCValue_temp/1024.0 * 5.0;
		
			double temperature = (voltage - 0.75) 100 + 25;
			double current = 
				
			if (current > 15){
				LATBbits.LATB1 = 1;
			}
			if else (current <= 15){
				LATBbits.LATB1 = 0;
			}
			sprintf (str,"$MCFBK,%.2f,%.2f*", current, temperature);
			//----------------------------------------------------->sending uart

	} // repeat
    
}

    
void scheduler(heartbeat schedInfo[]) {
    int i;
    for (i = 0; i < MAX_TASKS; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            schedInfo[i].f(schedInfo[i].params);
            /*switch (i) {
                case 0:
                    write_char_to_lcd();
                    break;
                case 1:
                    slide_controller();
                    break;
                case 2:
                    blink_led();
                    break;
                case 3:
                    check_button();
                    break;
                case 4:
                    read_potentiometer();
                    break;
            }*/
            schedInfo[i].n = 0;
        }
    }
}

void adc_configuration() { //using sequential sampling
	ADCON3bits.ADCS = 8; //----------> con 8 tad time troppo breve interferenza tra temp e corrente non switcha bene
	ADCON1bits.ASAM = 1; // manual sampling start
	ADCON1bits.SSRC = 7; // automatic conversion start
	ADCON3bits.SAMC = 31; // fixed conversion time (neededOnly if SSRC = 7) // could be 0 only only if multichannel multiple sequential time //16 troppo breve  e ci sono interferenze
	ADCON2bits.CHPS = 1; // CH0 & CH1
	ADCHSbits.CH0SA = 2; // AN2 connected to CH0
	ADCHSbits.CH123SA = 1; // AN3 connected to CH1
	ADPCFG = 0xFFFF;
	ADPCFGbits.PCFG2 = 0; // AN2 as analog
	ADPCFGbits.PCFG3 = 0; // AN3 as analog
	ADCON2bits.SMPI = 1; // 2 sample/convert sequences
}

int main(void) {
    heartbeat schedInfo[MAX_TASKS];
    
	ls structure;
	
    tmr_wait_ms(TIMER2, 1000);

    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0; D4
    TRISEbits.TRISE8 = 1;

    tmr_setup_period(TIMER1, 5); //------------------------------------------> control loop 200 hz = 1/(0.005 s )

	/////////// PWM //////////////

    tmr_setup_period(TIMER1, 5);
    PTCONbits.PTMOD = 0; // free running
    //PTCONbits.PTCKPS = 0; // 1:1 prescaler
	PWMCON1bits.PEN2H = 1;
    PWMCON1bits.PEN2L = 1;
	
	DTCON1bits.DTAPS = 0; //prescaler of the deat time --> 00 fot TCY
    DTCON1bits.DTA = 19; // approx 10 us
    
	
	PTCONbits.PTCKPS = 0; // 1:1 prescaler
	//PTCONbits.PTCKPS = 1; // 1:4 prescaler        
    PTPER = 1843; // 1 kHz
    //PTPER = 9216; // 50 Hz
	PTCONbits.PTEN = 1; // enable pwm
       
    // Tcy = 543 ns ------------> in the documentation says 33.33 ns, in case we want to use 543 we must recalculate
    
    double duty_cycle;
    //UART SETUP
    U2BRG = 11; //(7372800/4) / (16/9600)-1 at 9600 is the baud rate
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX
	/////////////////////////
    adc_configuration();
	
	
    schedInfo[0].n = 0;
    schedInfo[0].N = 1; //------------------------------------------> depends from baudrate 9600 bps ???????? N =
    schedInfo[0].f = read_spe_uart; 
    schedInfo[0].params = (void*)(&structure);
    schedInfo[1].n = 0;
    schedInfo[1].N = 30; //------------------------------------------> how frequently we want to change the speed? Bro I think blink_led has early deadline, and without reading the potentiometer
    schedInfo[1].f = set_voltage_DC;					// we change frequently a speed which is always the same? because higher priority than read_pot?
    schedInfo[1].params = (void*)(&structure); 
    schedInfo[2].n = 0;
    schedInfo[2].N = 40; //------------------------------------------> point 6, it is changing state at 1/(5*100)ms = 2Hz so it is blinking at 1Hz
    schedInfo[2].f = blink_led;
    schedInfo[2].params = NULL;
    schedInfo[3].n = 0;
    schedInfo[3].N = 40;
    schedInfo[3].f = temp_current;
    schedInfo[3].params = (void*)(&structure);
   
    
    
    
    structure.speed = 0;
    

    while (1) {
        scheduler(schedInfo);
        tmr_wait_period(TIMER1);
    }

    return 0;
}

