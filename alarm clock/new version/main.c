#include "app.h"

int main(void) {
    App app;
    App_Init(&app);
    App_MainLoop(&app);
    return 0;
}
