#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

typedef struct Button {
    volatile uint32_t* port;
    uint32_t mask;
    char locked;
    char pressed;
    void (*update)(struct Button* self, int debounce_ms);
} Button;

void Button_Init(Button* btn, volatile uint32_t* port, uint32_t mask);
void Button_Update(Button* btn, int debounce_ms);

#endif
