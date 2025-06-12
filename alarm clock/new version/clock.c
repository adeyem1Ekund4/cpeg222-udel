#include "clock.h"

void Clock_Init(Clock* clk) {
    clk->min1 = clk->min2 = clk->hour1 = clk->hour2 = 0;
    clk->increment = Clock_Increment;
    clk->decrement = Clock_Decrement;
    clk->reset = Clock_Reset;
}

void Clock_Increment(Clock* clk, int switched) {
    if (switched) { // minutes
        clk->min1++;
        if (clk->min1 > 9) {
            clk->min1 = 0;
            clk->min2++;
            if (clk->min2 > 5) {
                clk->min2 = 0;
            }
        }
    } else { // hours
        clk->hour1++;
        if (clk->hour1 > 9 && clk->hour2 == 0) {
            clk->hour1 = 0;
            clk->hour2++;
        } else if (clk->hour1 > 9 && clk->hour2 == 1) {
            clk->hour1 = 0;
            clk->hour2++;
        } else if (clk->hour1 > 3 && clk->hour2 == 2) {
            clk->hour1 = 0;
            clk->hour2 = 0;
        }
    }
}

void Clock_Decrement(Clock* clk, int switched) {
    if (switched) { // minutes
        if (clk->min2 != 0 && clk->min1 == 0) {
            clk->min1 = 9;
            clk->min2--;
        } else if (clk->min2 == 0 && clk->min1 == 0) {
            clk->min1 = 9;
            clk->min2 = 5;
        } else {
            clk->min1--;
        }
    } else { // hours
        if (clk->hour2 != 0 && clk->hour1 == 0) {
            clk->hour1 = 9;
            clk->hour2--;
        } else if (clk->hour2 == 0 && clk->hour1 == 0) {
            clk->hour2 = 2;
            clk->hour1 = 3;
        } else {
            clk->hour1--;
        }
    }
}

void Clock_Reset(Clock* clk) {
    clk->min1 = clk->min2 = clk->hour1 = clk->hour2 = 0;
}
