/*===================================CPEG222====================================
 * Program: Proj2Final.c
 * Authors:  Adeyemi Ekundayo and Zhaoqing,Zhu
 * Date:  10/9/2022
 * Description: This code uses an on-board BTN to control
 * the alarm, turning it on and off, and the SSD, which will display
 * the number of times btnC is pressed.
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
#include <stdio.h> 
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // NOTE: utils.c and utils.h must also be in your project 
//to use lcd.c
#include"ssd.h"

/* --------------------------- Forward Declarations-------------------------- */
void initialize_ports();
void initialize_output_states();
void handle_button_presses();
void double_buttons();


void delay_ms(int milliseconds);
void turnOnAlarm() ;
void turnOffAlarm() ;
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
#define BtnC PORTFbits.RF0
#define BtnU PORTBbits.RB1
#define BtnD PORTAbits.RA15
#define BtnL PORTBbits.RB0
#define BtnR PORTBbits.RB8

#define TRUE 1
#define FALSE 0
#define BUTTON_DEBOUNCE_DELAY_MS 20

/***** This section contains variables for the speaker ******/
#define TMR_FREQ_SINE   48000 // 48 kHz
// This array contains the values that implement one syne period, over 25 samples. 
// They are generated using this site: 
// http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
unsigned short rgSinSamples [] = {
256,320,379,431,472,499,511,507,488,453,
406,350,288,224,162,106, 59, 24,  5,  1,
 13, 40, 81,133,192};

#define RGSIN_SIZE  (sizeof(rgSinSamples) / sizeof(rgSinSamples[0]))

// the array of samples, to be stored when recording (Mode 2) and to be played when playing back (Mode 3).
unsigned short *pAudioSamples;

// global variables that store audio buffer position and size
int cntAudioBuf, idxAudioBuf;
/***** End of speaker declararions  ******/

typedef enum {ALARM_ON, ALARM_OFF} eModes ;
/* -------------------- Global Variable Declarations ------------------------ */

char buttonsLockedC = FALSE;
char buttonsLockedU = FALSE;
char buttonsLockedD = FALSE;
char buttonsLockedL = FALSE;
char buttonsLockedR = FALSE;

char pressedUnlockedBtnC = FALSE;
char pressedUnlockedBtnU = FALSE;
char pressedUnlockedBtnD = FALSE;
char pressedUnlockedBtnL = FALSE;
char pressedUnlockedBtnR = FALSE;


//eModes mode = ALARM_OFF ;
char mode = 1;
int x = 3;
int switched = 0;
int alarm=0;
int val = 0 ;
int val2 = 0;
int val3 = 0;
int val4 = 0;
int alarm_val = 0;
int alarm_val2 = 0;
int alarm_val3 = 0;
int alarm_val4 = 0;


/* ----------------------------- Main --------------------------------------- */
int main(void)
{
    /*-------------------- Port and State Initialization -------------------------
*/
    initialize_ports();
    initialize_output_states();
    
    SSD_WriteDigits(0,0,0,0,0,0,0,0);
    
    while (TRUE)
    {
        /*-------------------- Main logic and actions start 
--------------------------*/
        handle_button_presses();
        
        //SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
        
        switch(mode){
            case 1:
                /*----- Clock Time ----------*/
                LCD_WriteStringAtPos("   Set Clock   ", 1, 0);
                if (pressedUnlockedBtnL){
                    switched=!switched;
                }
            //MINUTES
                //increase value
                if(switched == 1){
                    if (pressedUnlockedBtnU){
                        val++;
                        if(val>9){
                            val=0;
                            val2++;
                            if(val2>5){
                                val=0;
                                val2=0;
                            }
                        }
                        SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                        
                    }
                    //decrease value
                    if (pressedUnlockedBtnD){
                        
                        
                        if (val2 != 0 && val == 0) {
                            val = 9;
                            val2--;
                            }
                        else if (val2 == 0 && val == 0){
                            val = 9;
                            val2 = 5;
                          
                        }
                        else {
                            val--;
                        }
                        SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                        
                        }
                }
            //HOURS
                //increase value
                if(switched==0){
                if (pressedUnlockedBtnU){
                    val3++;
                    if(val3 > 9 && val4 == 0){
                        val3=0;
                        val4++;
                        }
                    else if (val3 > 9 && val4 == 1){
                        val3=0;
                        val4++;
                                
                    }
                    else if (val3 > 3 && val4 == 2){
                        val3=0;
                        val4=0;
                    }
                    SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                    
                }
                //decrease value
                if (pressedUnlockedBtnD){
                    
                    if (val4 != 0 && val3 == 0) {
                        val3 = 9;
                        val4--;
                    
                    }
                    else if (val4 == 0 && val3 == 0){
                        val4=2;
                        val3=3;
                    }
                    
                    else{
                        val3--;
                    }   
                    SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                    
                    }
                }
                //RESET
                if(pressedUnlockedBtnR){
                    val=0;
                    val2=0;
                    val3=0;
                    val4=0;
                    SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                    
                }
                //go to mode 3
                if(pressedUnlockedBtnC){
                    mode = 2;
                    alarm_val=0;
                    alarm_val2=0;
                    alarm_val3=2;
                    alarm_val4=1;
                    SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                    
                }
                break;
                
            case 2:
                /*----- Alarm Time ----------*/
                LCD_WriteStringAtPos("   Set Alarm   ", 1, 0);
                
                
                if (pressedUnlockedBtnL){
                    switched=!switched;
                }
            //MINUTES
                //increase value
                if(switched == 1){
                    if (pressedUnlockedBtnU){
                        alarm_val++;
                        if(alarm_val>9){
                            alarm_val=0;
                            alarm_val2++;
                            if(alarm_val2>5){
                                alarm_val=0;
                                alarm_val2=0;
                            }
                        }
                        SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                        
                    }
                    //decrease value
                    if (pressedUnlockedBtnD){
                        
                        
                        if (alarm_val2 != 0 && alarm_val == 0) {
                            alarm_val = 9;
                            alarm_val2--;
                            }
                        else if (alarm_val2 == 0 && alarm_val == 0){
                            alarm_val = 9;
                            alarm_val2 = 5;
                          
                        }
                        else {
                            alarm_val--;
                        }
                        SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                        
                        }
                }
            //HOURS
                //increase value
                if(switched==0){
                if (pressedUnlockedBtnU){
                    alarm_val3++;
                    if(alarm_val3 > 9 && alarm_val4 == 0){
                        alarm_val3=0;
                        alarm_val4++;
                        }
                    else if (alarm_val3 > 9 && alarm_val4 == 1){
                        alarm_val3=0;
                        alarm_val4++;
                                
                    }
                    else if (alarm_val3 > 3 && alarm_val4 == 2){
                        alarm_val3=0;
                        alarm_val4=0;
                    }
                    SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                    
                }
                //decrease value
                if (pressedUnlockedBtnD){
                    
                    if (alarm_val4 != 0 && alarm_val3 == 0) {
                        alarm_val3 = 9;
                        alarm_val4--;
                    
                    }
                    else if (alarm_val4 == 0 && alarm_val3 == 0){
                        alarm_val4=2;
                        alarm_val3=3;
                    }
                    
                    else{
                        alarm_val3--;
                    }   
                    SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                    
                    }
                }
                

                //go to mode 3
                if(pressedUnlockedBtnC){
                    mode = 3;
                }
                //go back to mode 1
                if(pressedUnlockedBtnR){
                    mode = 1;
                }

                break;
            
            case 3:
                LCD_WriteStringAtPos("  Display Time   ", 1, 0);
                        //count up on the clock
                SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                delay_ms(1000);
                val++;
                if (val > 9) {
                    val = 0;
                    val2++;
                    
                    if (val2 > 5){
                        val2=0;
                        val3++;
                        
                        if (val3 > 9 && val4 == 0 ){
                            val3=0;
                            val4++;
                            
                        }
                        else if (val3 > 9 && val4 == 1){
                            val3=0;
                            val4++;
                            
                        }
                        else if (val3 > 3 && val4 == 2){
                            val3=0;
                            val4=0;
                            
                        }
                    }
                }
                
                if(val == alarm_val && val2 == alarm_val2 && val3 == alarm_val3 && val4 == alarm_val4){
                    mode = 4;
                
                }
                
                //go back to mode 2
                if(pressedUnlockedBtnC){
                    
                   mode = 2;
                }
                //go back to mode 1
                if(pressedUnlockedBtnR){
                   mode = 1;
                   val=0;
                   val2=0;
                   val3=0;
                   val4=0;
                   SSD_WriteDigits(val,val2,val3,val4,0,0,0,0);
                } 
                break;       
            case 4: 
                double_buttons();
               
                LCD_WriteStringAtPos("    Alarming        ", 1, 0);
                //mode=ALARM_ON;
                alarm++;
                turnOnAlarm();
                SSD_WriteDigits(alarm_val,alarm_val2,alarm_val3,alarm_val4,0,0,0,0);
                    delay_ms(365);
                SSD_WriteDigits(20,20,20,20,0,0,0,0);
                    delay_ms(365);
                
                if(pressedUnlockedBtnC && pressedUnlockedBtnR){
                        //mode=ALARM_OFF;
                        turnOffAlarm();
                        mode = 3;
                       
                    }
                    //if the buttons are not pressed with the next tens seconds, the alarm will turn off and go back to mode 3.
                else if(alarm ==10){
                    
                    turnOffAlarm();
                    mode=3;
                }
              

                break;

        /*--------------------- Action and logic end 
---------------------------------*/
    }
}
}
/* -------------------------- Function Definitions 
---------------------------------- */
void initialize_ports()
{
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO

    /* The following line sets the tristate of Port F bit 0 to 1.
     *  BtnC is connected to that pins. When the tristate of a pin is set high,
     *  the pin is configured as a digital input. */
    
    TRISBbits.TRISB1 = 1; // RB1 (BTNU) configured as input
    ANSELBbits.ANSB1 = 0; // RB1 (BTNU) disabled analog
    TRISBbits.TRISB0 = 1; // RB1 (BTNL) configured as input
    ANSELBbits.ANSB0 = 0; // RB1 (BTNL) disabled analog
    TRISFbits.TRISF4 = 1; // RF0 (BTNC) configured as input
    TRISBbits.TRISB8 = 1; // RB8 (BTNR) configured as input
    ANSELBbits.ANSB8 = 0; // RB8 (BTNR) disabled analog
    TRISAbits.TRISA15 = 1; // RA15 (BTND) configured as input
    
    LCD_Init(); // A library function provided by Digilent
    SSD_Init();    //SSD Init

    // the following lines configure interrupts to control the speaker
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
        /* The following code sets up the alarm timer and interrupts */
    tris_A_OUT = 0;    
    rp_A_OUT = 0x0C; // 1100 = OC1
        // disable analog (set pins as digital)
    ansel_A_OUT = 0;
    
    T3CONbits.TCKPS = 0;     //1:1 prescale value
    T3CONbits.TGATE = 0;     //not gated input (the default)
    T3CONbits.TCS = 0;       //PCBLK input (the default)
    
    OC1CONbits.ON = 0;       // Turn off OC1 while doing setup.
    OC1CONbits.OCM = 6;      // PWM mode on OC1; Fault pin is disabled
    OC1CONbits.OCTSEL = 1;   // Timer3 is the clock source for this Output Compare module
    
    IPC3bits.T3IP = 7;      // interrupt priority
    IPC3bits.T3IS = 3;      // interrupt subpriority
    
    macro_enable_interrupts();  // enable interrupts at CPU

}
void initialize_output_states()
{

    LCD_WriteStringAtPos("    Group 31     ", 0, 0); //Display "Welcome" at line 0, 
            //position 0, using spaces to center it and clear any previously 
            //displayed letters
    LCD_WriteStringAtPos("   Set Clock   ", 1, 0); //Display "Press BtnC" at line 
            //1, position 0, using spaces to center it and clear any
            //previously displayed letters
    turnOffAlarm();
}
/* This below function turns on the alarm*/
void turnOnAlarm()
{
    //set up alarm
    PR3 = (int)((float)((float)PB_FRQ/TMR_FREQ_SINE) + 0.5);               
    idxAudioBuf = 0;
    cntAudioBuf = RGSIN_SIZE;
    pAudioSamples = rgSinSamples;

    // load first value
    OC1RS = pAudioSamples[0];
    TMR3 = 0;

    T3CONbits.ON = 1;        //turn on Timer3
    OC1CONbits.ON = 1;       // Start the OC1 module  
    IEC0bits.T3IE = 1;      // enable Timer3 interrupt    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}
/* This below function turns off the alarm*/
void turnOffAlarm()
{
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
}
/* The below function only handles BtnC presses. Think about how it could be
 expanded to handle all button presses.*/
void handle_button_presses()
{
    
   
    pressedUnlockedBtnC = FALSE;
    pressedUnlockedBtnU = FALSE;
    pressedUnlockedBtnD = FALSE;
    pressedUnlockedBtnL = FALSE;
    pressedUnlockedBtnR = FALSE;
    
    if (BtnC && !buttonsLockedC)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = TRUE;
        pressedUnlockedBtnC = TRUE;
    }
    else if (!BtnC && buttonsLockedC)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = FALSE;
    }
    if (BtnU && !buttonsLockedU)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedU = TRUE;
        pressedUnlockedBtnU = TRUE;
    }
    else if (!BtnU && buttonsLockedU)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedU = FALSE;
    }
    if (BtnD && !buttonsLockedD)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedD = TRUE;
        pressedUnlockedBtnD = TRUE;
    }
    else if (!BtnD && buttonsLockedD)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedD = FALSE;
    }
    if (BtnL && !buttonsLockedL)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedL = TRUE;
        pressedUnlockedBtnL = TRUE;
    }
    else if (!BtnL && buttonsLockedL)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedL = FALSE;
    }
    if (BtnR && !buttonsLockedR)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedR = TRUE;
        pressedUnlockedBtnR = TRUE;
    }
    else if (!BtnR && buttonsLockedR)
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedR = FALSE;
    }
    
}

void double_buttons(){
    if ((BtnC && !buttonsLockedC) && (BtnR && !buttonsLockedR))
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = TRUE;
        buttonsLockedR = TRUE;
        pressedUnlockedBtnC = TRUE;
        pressedUnlockedBtnR = TRUE;
    }
    else if ((!BtnC && buttonsLockedC) && (!BtnR && buttonsLockedR))
    {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS); // debounce
        buttonsLockedC = TRUE;
        buttonsLockedR = TRUE;
        
    }
      
}


void delay_ms(int milliseconds)
{
    int i;
    for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++) 
    {}
}

/* ------------------------------------------------------------ */
/***	Timer3ISR
**
**	Description:
**		This is the interrupt handler for Timer3. According to each mode, it is called at specific frequencies, as initialized in AUDIO_Init.
    Mode 0 (Generate sound using sine) - Advance current index in the sine definition buffer, initialize OC1 with the current sine definition value.
**          
*/
void __ISR(_TIMER_3_VECTOR, IPL7AUTO) Timer3ISR(void) 
{  
   // play sine
    // load sine value into OC register
    OC1RS = 4*pAudioSamples[(++idxAudioBuf) % cntAudioBuf];
    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}
