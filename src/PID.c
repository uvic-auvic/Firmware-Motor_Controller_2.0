#include <stdlib.h>

#include "PID.h"

#define BIT_K(N, K) (N >> K) & 1

uint8_t zero_on_cross = 0b00000000;
uint16_t delay = 20;

static void PID_init(PID_controller_t *pid, uint32_t kP, uint32_t kI,
                uint32_t kD, uint16_t integral_limit, uint16_t epsilon)
{
        pid->P = kP;
        pid->I = kI;
        pid->D = kD;

        pid->integral_limit = integral_limit;
        pid->epsilon = epsilon;
}

static void PID_reset(PID_controller_t *pid)
{
        pid->error       = 0;
        pid->last_time   = 0;
        pid->dt          = 10; /* dt is set to 10 to avoid divide by 0 error */
        pid->error_sum   = 0;
        pid->last_error  = 0;
        pid->output      = 0;
        pid->last_output = 0;
}

static uint32_t PID_calculate(PID_controller_t *pid, uint8_t motor, uint32_t error)
{
        /* TODO: Use RTOS time instead of old RobotC time */
        pid->dt = nPgmTime - pid->lastTime;
        pid->lastTime = nPgmTime;

        pid->lastError = pid->error;
        pid->error = error;

        float d_error = pid->dt != 0 ? (pid->error - pid->last_error) / pid->dt : 0;

        if (pid->zeroOnCross && (sign(pid->error) != sign(pid->lastError))) {
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
