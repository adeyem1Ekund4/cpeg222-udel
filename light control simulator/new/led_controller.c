#include "led_controller.h"
#include <xc.h>

#define LED_ON 0xFF
#define LED_OFF 0x00

void LedController_update(LedController* self, int mode, int sw6, int sw7) {
    switch (mode) {
        case 1:
            self->pattern = LED_ON;
            self->state = 1;
            break;
        case 2:
            if (sw7 == 0)
                self->pattern <<= 1;
            else
                self->pattern >>= 1;
            if (self->pattern == 0)
                self->state = 0;
            break;
        case 3:
            self->pattern = LED_OFF;
            self->state = 0;
            self->set = 7;
            break;
        case 4:
            if (sw7 == 0)
                self->pattern = (self->pattern << 1) + 1;
            else {
                self->pattern |= (1 << self->set);
                self->set--;
            }
            if (self->pattern == LED_ON)
                self->state = 1;
            break;
    }
    LATA = self->pattern;
}

LedController LedController_create() {
    LedController c;
    c.state = 1;
    c.pattern = LED_ON;
    c.set = 7;
    c.update = LedController_update;
    return c;
}
