#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdbool.h>

typedef struct SensorData {
    bool ir1, ir2, ir3, ir4;
    int microphoneLevel;
    bool allSensorsBlocked;
    bool pathClear;
} SensorData;

typedef struct SensorManager {
    // Private data
    SensorData currentReadings;
    int soundThreshold;
    
    // Public methods
    void (*init)(struct SensorManager* self);
    void (*update)(struct SensorManager* self);
    SensorData (*getData)(struct SensorManager* self);
    bool (*isSoundDetected)(struct SensorManager* self);
    bool (*isPathBlocked)(struct SensorManager* self);
    int (*getPathDirection)(struct SensorManager* self); // -1 left, 0 straight, 1 right, 2 reverse
} SensorManager;

SensorManager* SensorManager_new(void);
void SensorManager_destroy(SensorManager* self);

#endif
