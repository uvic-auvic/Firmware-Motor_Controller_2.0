#ifndef PID_H_
#define PID_H_

#include "motors.h"

/**
 * Perform the PID calculations for all 8 motors.
 * This function should be called periodically
 */
extern void pid_update();

/**
 * Resets the provided PID controller
 */
extern void pid_reset(motors_t motor_x);

/**
 * Update the target RPM for the given motor
 */
extern void pid_update_rpm(motors_t motor_x, int16_t rpm);

extern void pid_update_motor_speed_percent(motors_t motor_x, uint16_t percent, direction_t dir);

#endif
