/*===================================CPEG222====================================
 * Program:      Project 4
 * Authors:     Adeyemi Ekundayo, Elvin Gunadasa, Zhu Zhaoqing
 * Date:        11/29/2022
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))
#define TMR_TIME .016

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "utils.h"
#include "ssd.h"
#include "srv.h"
#include "rgbled.h"
#include "pmods.h"
#include "mic.h"
#include "led.h"
#include "lcd.h"    // Digilent Library for using the on-board LCD
#include "i2c.h"
#include "config.h" // Basys MX3 configuration header
#include "acl.h" // Digilent Library for using the on-board accelerometer
#include "adc.h"

#define TRUE 1
#define FALSE 0

//Sensors
#define IR4 PORTDbits.RD9
#define IR3 PORTDbits.RD11
#define IR2 PORTDbits.RD10
#define IR1 PORTDbits.RD8

int array[120];

//Switches
#define SW0 PORTFbits.RF3  
#define SW1 PORTFbits.RF5  
#define SW6 PORTBbits.RB10  
#define SW7 PORTBbits.RB9

#define R4 LATCbits.LATC3
#define R3 LATGbits.LATG7
#define R2 LATGbits.LATG8
#define R1 LATGbits.LATG9

typedef enum _MODE {
    READY
}eModes;

eModes mode = READY;

//characters
char press = FALSE;
char count = FALSE;
char clap = 0;
char lineB = TRUE;

//integers
int sound = 0;
int x;
int t = 0;
int rr;
int modeX = 1;


//the subroutines
void CNConfig();
void delay_ms(int);
void display_error();
void initialize_ports();
void R_STOP();
void L_STOP();
void R_FORWARD();
void L_FORWARD();
void R_REV();
void L_REV();
void Timer3_Setup();
void CN_Handler();

int main(void){
	macro_enable_interrupts();
	DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO 
	SRV_Init();
    
	SSD_Init();
	MIC_Init();
	LED_Init();
	LCD_Init();
	ADC_Init();

    initialize_ports();

    TRISCbits.TRISC3 = 0;
    TRISGbits.TRISG7 = 0; 
    ANSELGbits.ANSG7 = 0; 
    TRISGbits.TRISG8 = 0; 
    ANSELGbits.ANSG8 = 0;
    TRISGbits.TRISG9 = 0; 
    ANSELGbits.ANSG9 = 0;
	
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1; 
    TRISCbits.TRISC4 = 1; 
    TRISGbits.TRISG6 = 1;
    ANSELGbits.ANSG6 = 0;
    	
	//ServoS
    	TRISBbits.TRISB8 = 0;
    	TRISAbits.TRISA15 = 0;
    	ANSELBbits.ANSB8 = 0;

        tris_SRV_S0PWM = 0;
    	tris_SRV_S1PWM = 0;
    	ansel_SRV_S0PWM = 0;
        
    	// Remap pins to OCs
    	rp_SRV_S0PWM = 0x0B; // 1011 = OC5
    	rp_SRV_S1PWM = 0x0B; // 1011 = OC4

        RPB8R = 0x0B; //servo 0 to 0C5
    	RPA15R = 0x0B; //servo 1 to OC4

        Timer3_Setup();
        //OC4 right
    	OC4CONbits.ON = 0; // Turn off OC4 while doing setup.
        OC4CONbits.OCM = 6; // PWM mode on OC4; Fault pin is disabled
        OC4CONbits.OCTSEL = 1; // Timer3 is the clock source
        OC4CONbits.ON = 1; // Start the OC1 module
        //OC5 left
        OC5CONbits.ON = 0; // Turn off OC4 while doing setup.
        OC5CONbits.OCM = 6; // PWM mode on OC4; Fault pin is disabled
        OC5CONbits.OCTSEL = 1; // Timer3 is the clock source
        OC5CONbits.ON = 1; // Start the OC1 module

	while(TRUE){
		LCD_WriteStringAtPos("   ROBOTROCK",0,0);
		while(sound == 0){
			t = -1;
			if(MIC_Val() > 950){
			delay_ms(10);
			while(t < 10){
				if(MIC_Val() > 950){
					sound++;
					t=-1;
					break;
				}
			}
		}
	}
	
	if((!IR1 && !IR2 && !IR3 && !IR4)||(IR1 && !IR2 && !IR3 && IR4)){//straight
		R_FORWARD();
		L_FORWARD();
        delay_ms(5);
        modeX =2;
	}
        
	if((IR1 && !IR2 && !IR3 && !IR4)||(IR1 && IR2 && !IR3 && !IR4)){//right
		R_STOP();
		L_FORWARD();
		modeX = 4;
        delay_ms(5);
	}
//	if(!IR1 && !IR2 && IR3 && IR4){
//		R_FORWARD();
//		L_STOP();
//		modeX = 4;
//        LCD_WriteStringAtPos("0011",1,7);
//        delay_ms(5);
//	}
//    if(!IR1 && IR2 && IR3 && IR4){
//		R_FORWARD();
//		L_STOP();
//		modeX = 4;
//        LCD_WriteStringAtPos("0111",1,7);
//        delay_ms(5);
//	}
	if((!IR1 && !IR2 && !IR3 && IR4)||(!IR1 && !IR2 && IR3 && IR4)){//left
		R_FORWARD();
		L_STOP();
		modeX = 3;       
        delay_ms(5);
	}
//	else if(IR1 && IR2 && !IR3 && !IR4){
//		R_STOP();
//		L_FORWARD();
//		modeX = 3;
//        LCD_WriteStringAtPos("1100",1,7);
//        delay_ms(5);
//	}
//	else if(IR1 && !IR2 && !IR3 && !IR4){
//		R_STOP();
//		L_FORWARD();
//		modeX = 3;
//        LCD_WriteStringAtPos("1000",1,7);
//        delay_ms(5);
//	}
	if(IR1 && IR2 && IR3 && IR4){//reverse
		R_REV();
		L_REV();
		modeX = 5;    
        delay_ms(5);
	}

	switch(modeX){
		case 1:
			count = FALSE;
			LCD_WriteStringAtPos("STP",1,0);
			LCD_WriteStringAtPos("STP",1,13);
			R_STOP();
			L_STOP();
			break;
		case 2:
			count = TRUE;
			LCD_WriteStringAtPos("FWD",1,0);
			LCD_WriteStringAtPos("FWD",1,13);
           
			break;
		case 3:
			count = TRUE;
			LCD_WriteStringAtPos("STP",1,0);
			LCD_WriteStringAtPos("FWD",1,13);
            
			break;
		case 4:
			count = TRUE;
			LCD_WriteStringAtPos("FWD",1,0);
			LCD_WriteStringAtPos("STP",1,13);
			break;
            
		case 5:
			count = TRUE;
			LCD_WriteStringAtPos("REV",1,0);
			LCD_WriteStringAtPos("REV",1,13);
			break;
	}

}


}
void initialize_ports() {
    DDPCONbits.JTAGEN = 0; 
    TRISA &= 0xFF00;
    LATA &= 0xFF00;
    TRISFbits.TRISF0 = 1;
    TRISBbits.TRISB8 = 1; //Right button is configured as input
    ANSELBbits.ANSB8 = 0; // Right button disabled analog
    TRISBbits.TRISB10 = 1; //switch 6 configured as input
    ANSELBbits.ANSB10 = 0; // switch 6 disabled analog
    TRISBbits.TRISB9 = 1; // RB9 (SW7) configured as input
    ANSELBbits.ANSB9 = 0; // RB9 (SW7) disabled analog
    
    LCD_Init(); // A library function provided by Digilent
}

void R_STOP(){
	OC4RS = (PR3/13.3);
}

void L_STOP(){
	OC5RS = (PR3/13.3);
}

void R_FORWARD(){
	OC4RS = (PR3/20);
}

void L_FORWARD(){
	OC5RS = (PR3/10);
}

void R_REV(){
	OC4RS = (PR3/10);
}

void L_REV(){
	OC5RS = (PR3/20);
}

void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;

    CNCONCbits.ON = 1; //all port D pins to trigger CN interrupts
    CNCONGbits.ON = 1; //all port D pins to trigger CN interrupts
    CNENC = 0x16;
    CNENG = 0x40;
    CNPUC = 0x16;
    CNPUG = 0x40;
    IPC8bits.CNIP = 0b111; // set CN priority to  5
    IPC8bits.CNIS = 0b11; // set CN sub-priority to 3
    IFS1bits.CNCIF = 0; //Clear interrupt flag status bit
    IFS1bits.CNGIF = 0;
    IEC1bits.CNCIE = 1;
    IEC1bits.CNGIE = 1;
    int j = PORTC; //read port to clear mismatch on CN pins
    int k = PORTG;
    macro_enable_interrupts(); // re-enable interrupts
}

void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
    IEC1bits.CNDIE = 0;
     
    int i;
    //if (count) {
    i++;
    //}

    int t;
    if (i % 5 == 0) {
        if (count) {
            t++;
            if (t < 10) {
                SSD_WriteDigits((t % 1000) % 100 % 10, 17, 17, 17, 0, 0, 0, 0);
            } else if (t < 100) {
                SSD_WriteDigits((t % 1000) % 100 % 10, (t % 1000) % 100 / 10, 17, 17, 0, 0, 0, 0);
            } else if (t < 1000) {
                SSD_WriteDigits((t % 1000) % 100 % 10, (t % 1000) % 100 / 10, (t % 1000) / 100, 17, 0, 0, 0, 0);
            } else {
                SSD_WriteDigits((t % 1000) % 100 % 10, (t % 1000) % 100 / 10, (t % 1000) / 100, t / 1000, 0, 0, 0, 0);
            }
        } else if (t != 0) {
            t = 0;
            SSD_WriteDigits(0, 17, 17, 17, 0, 0, 0, 0);
            // clear ssd with zero
        }

    }

    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC1bits.CNDIE = 1; //enable interrupts
}

void Timer3_Setup() {

    macro_disable_interrupts;
    PR3 = 780;

    TMR3 = 0; // initialize count to 0
    T3CONbits.TCKPS = 0b111; // 1:256? prescale value 7
    T3CONbits.TGATE = 0; // not gated input (default)
    T3CONbits.TCS = 0; // PCBLK input (default)
    T3CONbits.ON = 1; // turn on Timer3
    IPC3bits.T3IP = 0b111; // priority 7
    IPC3bits.T3IS = 0b11; // subpriority 3
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
    macro_enable_interrupts();
}

void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void) {

    IEC1bits.CNCIE = 0;
    IEC1bits.CNGIE = 0;
    int j = PORTC;
    int k = PORTG;
    IFS1bits.CNCIF = 0;
    IFS1bits.CNGIF = 0;
    IEC1bits.CNCIE = 1;
    IEC1bits.CNGIE = 1;
}

void delay_ms(int milliseconds) {
    int i;
    for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++) {
    }
}