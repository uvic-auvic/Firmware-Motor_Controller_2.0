#include <stdlib.h>
#include <stdint.h>

#include "PID.h"

#define MOTOR_1_P 0
#define MOTOR_2_P 0
#define MOTOR_3_P 0
#define MOTOR_4_P 0
#define MOTOR_5_P 0
#define MOTOR_6_P 0
#define MOTOR_7_P 0
#define MOTOR_8_P 0

#define MOTOR_1_I 0
#define MOTOR_2_I 0
#define MOTOR_3_I 0
#define MOTOR_4_I 0
#define MOTOR_5_I 0
#define MOTOR_6_I 0
#define MOTOR_7_I 0
#define MOTOR_8_I 0

#define MOTOR_1_D 0
#define MOTOR_2_D 0
#define MOTOR_3_D 0
#define MOTOR_4_D 0
#define MOTOR_5_D 0
#define MOTOR_6_D 0
#define MOTOR_7_D 0
#define MOTOR_8_D 0

#define MOTOR_1_INT_LIM 0
#define MOTOR_2_INT_LIM 0
#define MOTOR_3_INT_LIM 0
#define MOTOR_4_INT_LIM 0
#define MOTOR_5_INT_LIM 0
#define MOTOR_6_INT_LIM 0
#define MOTOR_7_INT_LIM 0
#define MOTOR_8_INT_LIM 0

#define MOTOR_1_EPSILON 0
#define MOTOR_2_EPSILON 0
#define MOTOR_3_EPSILON 0
#define MOTOR_4_EPSILON 0
#define MOTOR_5_EPSILON 0
#define MOTOR_6_EPSILON 0
#define MOTOR_7_EPSILON 0
#define MOTOR_8_EPSILON 0

#define ZERO_ON_CROSS 0b00000000
#define PID_DELAY 20

static void PID_reset(PID_controller_t *pid)
{
        pid->error       = 0;
        pid->error_sum   = 0;
        pid->last_error  = 0;
        pid->output      = 0;
        pid->last_output = 0;
        pid->last_time   = 0;
        pid->dt          = PID_DELAY; /* Avoid div by 0 */
}

static uint32_t PID_calculate(PID_controller_t *pid, uint8_t motor, uint32_t error)
{
        /* TODO: Use RTOS time instead of old RobotC time */
        pid->dt = nPgmTime - pid->lastTime;
        pid->lastTime = nPgmTime;

        pid->last_error = pid->error;
        pid->error = error;

        float d_error = pid->dt != 0 ? (pid->error - pid->last_error) / pid->dt : 0;

        if (pid->zeroOnCross && (sign(pid->error) != sign(pid->last_error))) {
                pid->error_sum = 0;
        }

        pid->output = pid->P * pid->error + pid->D * d_error;

        if (abs(pid->output) < MAX_SPEED) {
                pid->error_sum = abs(pid->error) > pid->epsilon ? pid->error_sum + pid->error * pid->dt : 0;
        }

        pid->error_sum = clamp(pid->error_sum, pid->integral_limit);
        pid->output += pid->I * pid->error_sum;

        return PIDFilter(pid);
}
