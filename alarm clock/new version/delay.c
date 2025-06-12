#include "delay.h"
#define LOOPS_NEEDED_TO_DELAY_ONE_MS 1426

void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * LOOPS_NEEDED_TO_DELAY_ONE_MS; ++i) {}
}
