#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

typedef struct DisplayManager {
    // Private data
    char lcdLine1[17];
    char lcdLine2[17];
    int currentCount;
    bool countingEnabled;
    
    // Public methods
    void (*init)(struct DisplayManager* self);
    void (*updateLCD)(struct DisplayManager* self, const char* line1, const char* line2);
    void (*updateSSD)(struct DisplayManager* self, int value);
    void (*showMotorStatus)(struct DisplayManager* self, const char* left, const char* right);
    void (*enableCounting)(struct DisplayManager* self, bool enable);
    void (*incrementCounter)(struct DisplayManager* self);
    void (*resetCounter)(struct DisplayManager* self);
} DisplayManager;

DisplayManager* DisplayManager_new(void);
void DisplayManager_destroy(DisplayManager* self);

#endif
