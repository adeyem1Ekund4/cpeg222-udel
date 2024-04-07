/*===================================CPEG222====================================
 * Program: Proj1_Template.c
 * Authors:  Adeyemi Ekundayo and Zhaoqing Zhu
 * Date:  9/22/2022
 * Description: This template uses an on-board BTN to control
 * the on-board LEDs 0 through 7. All LEDs are intially off.
 * Input: Button press.
 * Output: All LEDs are turned on and off.
==============================================================================*/
/*---- Board system settings. PLEASE DO NOT MODIFY THIS PART FOR PROJECT 1 ---*/
#ifndef _SUPPRESS_PLIB_WARNING //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider 
//(PLL Divide by 1)
#pragma config FNOSC = PRIPLL   // Oscillator Selection Bits (Primary Osc w/PLL 
//(XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF    // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT     // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8   // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#include <xc.h>     //Microchip XC processor header which links to the 
//PIC32MX370512L header
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // NOTE: utils.c and utils.h must also be in your project 
//to use lcd.c
/* --------------------------- Forward Declarations-------------------------- */
void initialize_ports();
void initialize_output_states();
void toggle_half_LEDs();
void handle_button_presses();
void delay_ms(int milliseconds);
/* ------------------------ Constant Definitions ---------------------------- */
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))
/* The Basys reference manual shows to which pin of the processor every IO 
connects. 
 BtnC connects to Port F pin 0. PORTF reads output values from Port F pins.
 LATF would be used to read input values on Port F pins and TRISF would be used to
 set tristate values of Port F pins. We will see LAT and TRIS later. */
#define BtnR_RAW PORTBbits.RB8
#define TRUE 1
#define FALSE 0
#define BUTTON_DEBOUNCE_DELAY_MS 20
#define SW6 PORTBbits.RB10
#define SW7 PORTBbits.RB9

int Mode = 0;
/* -------------------- Global Variable Declarations ------------------------ */
char buttonsLocked = FALSE;
char pressedUnlockedBtnR = FALSE;
int LED_OFF = 0b000000000;
int LED_ON = 0b11111111;

int j = 0;
int set = 7;/*--Flip--*/


/* ----------------------------- Main --------------------------------------- */
int main(void) {
    /*-------------------- Port and State Initialization -------------------------
     */
    initialize_ports();
    initialize_output_states();
    
    LATA = LED_ON;
    Mode = 1;
    while (TRUE) {
        /*-------------------- Main logic and actions start 
--------------------------*/
        handle_button_presses();
        switch (Mode) {
            case 0:
                break;
            case 1:
                LATA = LED_ON;
                LCD_WriteStringAtPos("   Mode 1   ", 1, 0);
                break;
            case 2:
                LCD_WriteStringAtPos("   Mode 2   ", 1, 0);
                if (SW7 == 0) {
                    
                        LATA = LATA << 1;
                        if(SW6 == 0){
                            delay_ms(1000);
                        }
                        else {
                            delay_ms(500);
                        }
                        
                        
                    
                } 
                if (SW7 == 1) {
                    
                        LATA = LATA >> 1;
                        if(SW6 == 0){
                            delay_ms(1000);
                        }
                        else {
                            delay_ms(500);
                        }
                        
                        
                    
                }
                if (LATA == LED_OFF){
                    Mode = 3;
                }
                break;
            case 3:
                LATA = LED_OFF;
                LCD_WriteStringAtPos("   Mode 3   ", 1, 0);
                set = 7;
                break;
            case 4:
                
                LCD_WriteStringAtPos("   Mode 4   ", 1, 0);
                if (SW7 == 0) {
                    
                        LATA = (LATA << 1) + 1;
                        if(SW6 == 0){
                            delay_ms(1000);
                        }
                        else{
                            delay_ms(500);
                        }
                        
          
                }
                if (SW7 == 1) {
                    
                        LATA = (LATA | 1 << set);
                        set = set - 1;
                        if(set < 0){
                            Mode = 4;
                        
                        
                        }
                        if(SW6 == 0){
                            delay_ms(1000);
                        }
                        else{
                            delay_ms(500);
                        }
                 
                }
                if (LATA == LED_ON){
                    Mode = 1;
                } 
                break;
        }
   
        if (pressedUnlockedBtnR) {
            if(Mode == 1){
                if(pressedUnlockedBtnR && LATA == LED_ON){
                    Mode = 2;
            }
            
            }
            else if(Mode == 3){
                if(pressedUnlockedBtnR && LATA == LED_OFF){
                    Mode = 4;
            }
            
            }
            
            
        }
        
        
        
    /*--------------------- Action and logic end 
    ---------------------------------*/
    }
}


/* -------------------------- Function Definitions 
---------------------------------- */
void initialize_ports() {
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO
    /* The following line sets the tristate of Port A bits 0-7 to 0. The LEDs are 
connected
     to those pins. When the tristate of a pin is set low, the pin is configured
     as a digital output. Notice an &= is used in conjunction with leading 1s
     (the FF) in order to keep the other bits of Port A (8-15) in their current 
state. */
    TRISA &= 0xFF00;
    /* The following line sets the tristate of Port F bit 0 to 1. BtnC is connected
to that
     pins. When the tristate of a pin is set high, the pin is configured as a
     digital input. */
    TRISBbits.TRISB8 = 1;//Button R Input
    ANSELBbits.ANSB8 = 0;//Disable Analog
    ANSELBbits.ANSB9 = 0;//Switch 7 disable analog
    ANSELBbits.ANSB10 = 0;//Switch 6 disable analog
    TRISBbits.TRISB9 = 1;//Switch 7 Input
    TRISBbits.TRISB10 = 1;//Switch 6 Input
    LCD_Init(); // A library function provided by Digilent
}

void initialize_output_states() {
    /* The following line sets the latch values of Port A bits 0-7 to 0. The LEDs 
are connected
     to those pins. When the latch of an LED output pin is set low, the LED is
     turned off. Notice we again used an &= in conjunction with leading 1s in order
     to keep the other latch values of Port A (bits 8-15) in their current state. 
     */
    LATA &= 0xFF00;
    LCD_WriteStringAtPos("    Group 31    ", 0, 0); //Display "Welcome" at line 0, 
    //position 0, using spaces to center it and clear any previously 
    //displayed letters
    LCD_WriteStringAtPos("   PressBtnR   ", 1, 0); //Display "Press BtnC" at line 
    

    //1, position 0, using spaces to center it and clear any
    //previously displayed letters
}

void toggle_half_LEDs() {
    /* Here we invert the latch values of Port A bits 0-7. The LEDs are connected
     to those pins. Again, bitwise operations allow us to change those bits
     without affecting the other 8 bits of Port A. */
    short topEigthLATABits = (LATA & 0xFF00);
    short invertertedBottomEightLATABits = (~LATA & 0x00AA);
    LATA = topEigthLATABits | invertertedBottomEightLATABits;
}

/* The below function only handles BtnC presses. Think about how it could be
 expanded to handle all button presses.*/
void handle_button_presses() {
    pressedUnlockedBtnR = FALSE;
    if (BtnR_RAW && !buttonsLocked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLocked = TRUE;
        pressedUnlockedBtnR = TRUE;
    } else if (!BtnR_RAW && buttonsLocked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLocked = FALSE;
    }
}

void delay_ms(int milliseconds) {
    int i;
    for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++) {
    }
}
