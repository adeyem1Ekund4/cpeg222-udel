#include "button.h"
#include "delay.h" // for delay_ms()

void Button_Init(Button* btn, volatile uint32_t* port, uint32_t mask) {
    btn->port = port;
    btn->mask = mask;
    btn->locked = 0;
    btn->pressed = 0;
    btn->update = Button_Update;
}

void Button_Update(Button* btn, int debounce_ms) {
    if ((*(btn->port) & btn->mask) && !btn->locked) {
        delay_ms(debounce_ms);
        btn->locked = 1;
        btn->pressed = 1;
    } else if (!(*(btn->port) & btn->mask) && btn->locked) {
        delay_ms(debounce_ms);
        btn->locked = 0;
        btn->pressed = 0;
    } else {
        btn->pressed = 0;
    }
}
