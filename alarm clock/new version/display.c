#include "display.h"
#include "ssd.h"
#include "lcd.h"

void Display_ShowClock(const Clock* clk) {
    SSD_WriteDigits(clk->min1, clk->min2, clk->hour1, clk->hour2, 0, 0, 0, 0);
}

void Display_LCD(const char* line0, const char* line1) {
    LCD_WriteStringAtPos(line0, 0, 0);
    LCD_WriteStringAtPos(line1, 1, 0);
}
