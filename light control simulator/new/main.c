#include "hardware.h"
#include "led_controller.h"
#include "button.h"
#include "delay.h"
#include "lcd.h"

int main(void) {
    initialize_ports();
    initialize_output_states();

    LedController leds = LedController_create();
    Button btnR = Button_create(BtnR_read_raw);

    int mode = 1;

    while (1) {
        btnR.update(&btnR);

        leds.update(&leds, mode, SW6, SW7);

        switch (mode) {
            case 1: LCD_WriteStringAtPos("   Mode 1   ", 1, 0); break;
            case 2: LCD_WriteStringAtPos("   Mode 2   ", 1, 0); break;
            case 3: LCD_WriteStringAtPos("   Mode 3   ", 1, 0); break;
            case 4: LCD_WriteStringAtPos("   Mode 4   ", 1, 0); break;
        }

        if (btnR.pressed) {
            if (mode == 1 && leds.state == 1)
                mode = 2;
            else if (mode == 3 && leds.state == 0)
                mode = 4;
        }
        if (mode == 2 && leds.state == 0)
            mode = 3;
        if (mode == 4 && leds.state == 1)
            mode = 1;

        if (mode == 2 || mode == 4) {
            if (SW6 == 0)
                delay_ms(1000);
            else
                delay_ms(500);
        }
    }
}
