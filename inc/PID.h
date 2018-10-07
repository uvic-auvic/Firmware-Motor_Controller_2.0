#ifndef PID_H_
#define PID_H_

#include "motors.h"

/**
 * Perform the PID calculations for all 8 motors.
 * This function should be called periodically
 */
extern void pid_update();

/**
 * Update the target RPM for the given motor
 */
extern void update_rpm(motors_t motor_x, uint32_t rpm);

#endif
