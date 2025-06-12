#include "app.h"
#include "display.h"
#include "delay.h"
#include "hardware_init.h" // you define this for your hardware setup

#define BUTTON_DEBOUNCE_DELAY_MS 20

void App_Init(App* app) {
    Hardware_Init(); // your function for TRIS/ANSEL/LCD/SSD etc.

    Clock_Init(&app->clock);
    Clock_Init(&app->alarm);
    // Set default alarm value
    app->alarm.hour2 = 1; app->alarm.hour1 = 2;
    app->alarm.min2 = 0; app->alarm.min1 = 0;

    Button_Init(&app->btnC, &PORTF, 0x1); // RF0
    Button_Init(&app->btnU, &PORTB, 0x2); // RB1
    Button_Init(&app->btnD, &PORTA, 0x8000); // RA15
    Button_Init(&app->btnL, &PORTB, 0x1); // RB0
    Button_Init(&app->btnR, &PORTB, 0x100); // RB8

    app->mode = MODE_SET_CLOCK;
    app->switched = 0;
    app->alarm_counter = 0;
}

static int Clock_Equals(const Clock* a, const Clock* b) {
    return a->min1 == b->min1 && a->min2 == b->min2 &&
           a->hour1 == b->hour1 && a->hour2 == b->hour2;
}

void App_MainLoop(App* app) {
    while (1) {
        app->btnC.update(&app->btnC, BUTTON_DEBOUNCE_DELAY_MS);
        app->btnU.update(&app->btnU, BUTTON_DEBOUNCE_DELAY_MS);
        app->btnD.update(&app->btnD, BUTTON_DEBOUNCE_DELAY_MS);
        app->btnL.update(&app->btnL, BUTTON_DEBOUNCE_DELAY_MS);
        app->btnR.update(&app->btnR, BUTTON_DEBOUNCE_DELAY_MS);

        switch (app->mode) {
            case MODE_SET_CLOCK:
                Display_LCD("   Set Clock   ", "Use buttons");
                if (app->btnL.pressed) {
                    app->switched = !app->switched;
                }
                if (app->btnU.pressed) {
                    app->clock.increment(&app->clock, app->switched);
                }
                if (app->btnD.pressed) {
                    app->clock.decrement(&app->clock, app->switched);
                }
                if (app->btnR.pressed) {
                    app->clock.reset(&app->clock);
                }
                if (app->btnC.pressed) {
                    app->mode = MODE_SET_ALARM;
                }
                Display_ShowClock(&app->clock);
                break;

            case MODE_SET_ALARM:
                Display_LCD("   Set Alarm   ", "Use buttons");
                if (app->btnL.pressed) {
                    app->switched = !app->switched;
                }
                if (app->btnU.pressed) {
                    app->alarm.increment(&app->alarm, app->switched);
                }
                if (app->btnD.pressed) {
                    app->alarm.decrement(&app->alarm, app->switched);
                }
                if (app->btnR.pressed) {
                    app->mode = MODE_SET_CLOCK;
                }
                if (app->btnC.pressed) {
                    app->mode = MODE_RUN;
                }
                Display_ShowClock(&app->alarm);
                break;

            case MODE_RUN:
                Display_LCD("  Display Time ", "");
                Display_ShowClock(&app->clock);
                delay_ms(1000);
                app->clock.increment(&app->clock, 1); // increment minutes
                if (Clock_Equals(&app->clock, &app->alarm)) {
                    app->mode = MODE_ALARM;
                    app->alarm_counter = 0;
                }
                if (app->btnC.pressed) app->mode = MODE_SET_ALARM;
                if (app->btnR.pressed) {
                    app->mode = MODE_SET_CLOCK;
                    app->clock.reset(&app->clock);
                }
                break;

            case MODE_ALARM:
                Display_LCD("    Alarming   ", "");
                // turnOnAlarm();
                Display_ShowClock(&app->alarm);
                delay_ms(365);
                // SSD_WriteDigits(20,20,20,20,0,0,0,0); // Alarm effect
                delay_ms(365);
                app->alarm_counter++;
                if (app->btnC.pressed && app->btnR.pressed) {
                    // turnOffAlarm();
                    app->mode = MODE_RUN;
                } else if (app->alarm_counter >= 10) {
                    // turnOffAlarm();
                    app->mode = MODE_RUN;
                }
                break;
        }
    }
}
