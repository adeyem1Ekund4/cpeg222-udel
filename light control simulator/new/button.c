#include "button.h"
#include "delay.h"

#define BUTTON_DEBOUNCE_DELAY_MS 20

void Button_update(Button* btn) {
    btn->pressed = 0;
    if (btn->read_raw() && !btn->locked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS);
        btn->locked = 1;
        btn->pressed = 1;
    } else if (!btn->read_raw() && btn->locked) {
        delay_ms(BUTTON_DEBOUNCE_DELAY_MS);
        btn->locked = 0;
    }
}

Button Button_create(int (*read_raw_func)()) {
    Button b;
    b.locked = 0;
    b.pressed = 0;
    b.read_raw = read_raw_func;
    b.update = Button_update;
    return b;
}
