#include "hardware.h"
#include <xc.h>
#include "lcd.h"

void initialize_ports() {
    DDPCONbits.JTAGEN = 0;
    TRISA &= 0xFF00;
    TRISBbits.TRISB8 = 1;
    ANSELBbits.ANSB8 = 0;
    ANSELBbits.ANSB9 = 0;
    ANSELBbits.ANSB10 = 0;
    TRISBbits.TRISB9 = 1;
    TRISBbits.TRISB10 = 1;
    LCD_Init();
}

void initialize_output_states() {
    LATA &= 0xFF00;
    LCD_WriteStringAtPos("    Group 31    ", 0, 0);
    LCD_WriteStringAtPos("   PressBtnR   ", 1, 0);
}

int BtnR_read_raw() {
    return PORTBbits.RB8;
}
