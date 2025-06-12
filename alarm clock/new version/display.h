#ifndef DISPLAY_H
#define DISPLAY_H

#include "clock.h"

void Display_ShowClock(const Clock* clk);
void Display_LCD(const char* line0, const char* line1);

#endif
