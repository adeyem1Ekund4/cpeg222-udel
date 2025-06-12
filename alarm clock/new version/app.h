#ifndef APP_H
#define APP_H

#include "button.h"
#include "clock.h"

typedef enum {
    MODE_SET_CLOCK,
    MODE_SET_ALARM,
    MODE_RUN,
    MODE_ALARM
} AppMode;

typedef struct {
    Clock clock;
    Clock alarm;
    Button btnC, btnU, btnD, btnL, btnR;
    AppMode mode;
    int switched;
    int alarm_counter;
} App;

void App_Init(App* app);
void App_MainLoop(App* app);

#endif
