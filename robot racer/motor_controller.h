#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

typedef enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} MotorDirection;

typedef struct MotorController {
    // Private data
    bool leftMotorEnabled;
    bool rightMotorEnabled;
    MotorDirection leftDirection;
    MotorDirection rightDirection;
    
    // Public methods
    void (*init)(struct MotorController* self);
    void (*setLeftMotor)(struct MotorController* self, MotorDirection dir);
    void (*setRightMotor)(struct MotorController* self, MotorDirection dir);
    void (*stopAll)(struct MotorController* self);
    void (*moveForward)(struct MotorController* self);
    void (*moveReverse)(struct MotorController* self);
    void (*turnLeft)(struct MotorController* self);
    void (*turnRight)(struct MotorController* self);
} MotorController;

MotorController* MotorController_new(void);
void MotorController_destroy(MotorController* self);

#endif
