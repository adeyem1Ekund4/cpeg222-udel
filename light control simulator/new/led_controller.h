#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

typedef struct LedController {
    int state;
    int pattern;
    int set;
    void (*update)(struct LedController*, int mode, int sw6, int sw7);
} LedController;

LedController LedController_create();
void LedController_update(LedController* self, int mode, int sw6, int sw7);

#endif
