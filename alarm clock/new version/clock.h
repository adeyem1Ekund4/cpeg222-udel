#ifndef CLOCK_H
#define CLOCK_H

typedef struct Clock {
    int min1, min2, hour1, hour2;
    void (*increment)(struct Clock* self, int switched); // switched: 0=hour, 1=min
    void (*decrement)(struct Clock* self, int switched);
    void (*reset)(struct Clock* self);
} Clock;

void Clock_Init(Clock* clk);
void Clock_Increment(Clock* clk, int switched);
void Clock_Decrement(Clock* clk, int switched);
void Clock_Reset(Clock* clk);

#endif
