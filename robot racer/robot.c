#include "robot.h"
#include "motor_controller.h"
#include "sensor_manager.h"
#include "display_manager.h"
#include <stdlib.h>
#include <stdio.h>

// Forward declarations of private methods
static void Robot_init(Robot* self);
static void Robot_update(Robot* self);
static void Robot_processSound(Robot* self);
static void Robot_processSensors(Robot* self);
static void Robot_move(Robot* self, MovementCommand cmd);
static void Robot_stop(Robot* self);
static void Robot_displayStatus(Robot* self);
static RobotState Robot_getState(Robot* self);
static void Robot_setState(Robot* self, RobotState state);

// Private helper methods
static void Robot_updateDisplay(Robot* self);
static void Robot_handleMovement(Robot* self, int direction);

// Robot constructor
Robot* Robot_new(void) {
    Robot* self = (Robot*)malloc(sizeof(Robot));
    if (!self) return NULL;
    
    // Initialize data members
    self->currentState = ROBOT_READY;
    self->soundTriggerLevel = 950;
    self->isActive = false;
    self->timerCount = 0;
    
    // Initialize sensors
    self->sensors.ir1 = false;
    self->sensors.ir2 = false;
    self->sensors.ir3 = false;
    self->sensors.ir4 = false;
    
    // Assign method pointers
    self->init = Robot_init;
    self->update = Robot_update;
    self->processSound = Robot_processSound;
    self->processSensors = Robot_processSensors;
    self->move = Robot_move;
    self->stop = Robot_stop;
    self->displayStatus = Robot_displayStatus;
    self->getState = Robot_getState;
    self->setState = Robot_setState;
    
    return self;
}

// Robot destructor
void Robot_destroy(Robot* self) {
    if (self) {
        free(self);
    }
}

// Private method implementations
static void Robot_init(Robot* self) {
    if (!self) return;
    
    // Initialize hardware components
    macro_enable_interrupts();
    DDPCONbits.JTAGEN = 0;
    
    // Initialize subsystems
    SRV_Init();
    SSD_Init();
    MIC_Init();
    LED_Init();
    LCD_Init();
    ADC_Init();
    
    // Initialize ports and timers
    initialize_ports();
    Timer3_Setup();
    
    self->currentState = ROBOT_READY;
    self->isActive = false;
}

static void Robot_update(Robot* self) {
    if (!self) return;
    
    self->processSound(self);
    
    if (self->isActive) {
        self->processSensors(self);
        self->displayStatus(self);
    }
}

static void Robot_processSound(Robot* self) {
    if (!self) return;
    
    static int soundTimer = 0;
    
    if (!self->isActive) {
        if (MIC_Val() > self->soundTriggerLevel) {
            delay_ms(10);
            soundTimer = 0;
            while (soundTimer < 10) {
                if (MIC_Val() > self->soundTriggerLevel) {
                    self->isActive = true;
                    self->setState(self, ROBOT_MOVING_FORWARD);
                    break;
                }
                soundTimer++;
            }
        }
    }
}

static void Robot_processSensors(Robot* self) {
    if (!self) return;
    
    // Read IR sensors
    self->sensors.ir1 = IR1;
    self->sensors.ir2 = IR2;
    self->sensors.ir3 = IR3;
    self->sensors.ir4 = IR4;
    
    // Determine movement direction based on sensor readings
    int direction = 0; // 0 = straight, -1 = left, 1 = right, 2 = reverse
    
    if ((!self->sensors.ir1 && !self->sensors.ir2 && !self->sensors.ir3 && !self->sensors.ir4) ||
        (self->sensors.ir1 && !self->sensors.ir2 && !self->sensors.ir3 && self->sensors.ir4)) {
        direction = 0; // Straight
    }
    else if ((self->sensors.ir1 && !self->sensors.ir2 && !self->sensors.ir3 && !self->sensors.ir4) ||
             (self->sensors.ir1 && self->sensors.ir2 && !self->sensors.ir3 && !self->sensors.ir4)) {
        direction = 1; // Right
    }
    else if ((!self->sensors.ir1 && !self->sensors.ir2 && !self->sensors.ir3 && self->sensors.ir4) ||
             (!self->sensors.ir1 && !self->sensors.ir2 && self->sensors.ir3 && self->sensors.ir4)) {
        direction = -1; // Left
    }
    else if (self->sensors.ir1 && self->sensors.ir2 && self->sensors.ir3 && self->sensors.ir4) {
        direction = 2; // Reverse
    }
    
    Robot_handleMovement(self, direction);
}

static void Robot_handleMovement(Robot* self, int direction) {
    if (!self) return;
    
    switch (direction) {
        case 0: // Straight
            self->move(self, MOVE_FORWARD);
            self->setState(self, ROBOT_MOVING_FORWARD);
            break;
        case -1: // Left
            self->move(self, TURN_LEFT);
            self->setState(self, ROBOT_TURNING_LEFT);
            break;
        case 1: // Right
            self->move(self, TURN_RIGHT);
            self->setState(self, ROBOT_TURNING_RIGHT);
            break;
        case 2: // Reverse
            self->move(self, MOVE_REVERSE);
            self->setState(self, ROBOT_REVERSING);
            break;
        default:
            self->stop(self);
            self->setState(self, ROBOT_STOPPED);
            break;
    }
}

static void Robot_move(Robot* self, MovementCommand cmd) {
    if (!self) return;
    
    switch (cmd) {
        case MOVE_FORWARD:
            R_FORWARD();
            L_FORWARD();
            break;
        case MOVE_REVERSE:
            R_REV();
            L_REV();
            break;
        case TURN_LEFT:
            R_FORWARD();
            L_STOP();
            break;
        case TURN_RIGHT:
            R_STOP();
            L_FORWARD();
            break;
        case MOVE_STOP:
        default:
            R_STOP();
            L_STOP();
            break;
    }
    delay_ms(5);
}

static void Robot_stop(Robot* self) {
    if (!self) return;
    self->move(self, MOVE_STOP);
    self->setState(self, ROBOT_STOPPED);
}

static void Robot_displayStatus(Robot* self) {
    if (!self) return;
    
    const char* leftStatus = "STP";
    const char* rightStatus = "STP";
    
    switch (self->currentState) {
        case ROBOT_MOVING_FORWARD:
            leftStatus = "FWD";
            rightStatus = "FWD";
            break;
        case ROBOT_TURNING_LEFT:
            leftStatus = "STP";
            rightStatus = "FWD";
            break;
        case ROBOT_TURNING_RIGHT:
            leftStatus = "FWD";
            rightStatus = "STP";
            break;
        case ROBOT_REVERSING:
            leftStatus = "REV";
            rightStatus = "REV";
            break;
        default:
            leftStatus = "STP";
            rightStatus = "STP";
            break;
    }
    
    LCD_WriteStringAtPos(leftStatus, 1, 0);
    LCD_WriteStringAtPos(rightStatus, 1, 13);
}

static RobotState Robot_getState(Robot* self) {
    return self ? self->currentState : ROBOT_READY;
}

static void Robot_setState(Robot* self, RobotState state) {
    if (self) {
        self->currentState = state;
    }
}
