#ifndef BUTTON_H
#define BUTTON_H

typedef struct Button {
    char locked;
    char pressed;
    int (*read_raw)();
    void (*update)(struct Button*);
} Button;

Button Button_create(int (*read_raw_func)());
void Button_update(Button* btn);

#endif
