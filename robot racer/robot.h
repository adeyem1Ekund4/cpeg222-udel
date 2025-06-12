#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>

// Robot states enumeration
typedef enum {
    ROBOT_READY,
    ROBOT_MOVING_FORWARD,
    ROBOT_TURNING_LEFT,
    ROBOT_TURNING_RIGHT,
    ROBOT_REVERSING,
    ROBOT_STOPPED
} RobotState;

// Robot movement directions
typedef enum {
    MOVE_STOP,
    MOVE_FORWARD,
    MOVE_REVERSE,
    TURN_LEFT,
    TURN_RIGHT
} MovementCommand;

// Robot class structure
typedef struct Robot {
    // Private data members
    RobotState currentState;
    int soundTriggerLevel;
    bool isActive;
    int timerCount;
    
    // IR sensor readings
    struct {
        bool ir1, ir2, ir3, ir4;
    } sensors;
    
    // Method pointers (public interface)
    void (*init)(struct Robot* self);
    void (*update)(struct Robot* self);
    void (*processSound)(struct Robot* self);
    void (*processSensors)(struct Robot* self);
    void (*move)(struct Robot* self, MovementCommand cmd);
    void (*stop)(struct Robot* self);
    void (*displayStatus)(struct Robot* self);
    RobotState (*getState)(struct Robot* self);
    void (*setState)(struct Robot* self, RobotState state);
} Robot;

// Constructor and destructor
Robot* Robot_new(void);
void Robot_destroy(Robot* self);

#endif
