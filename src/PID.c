#include <stdlib.h>
#include <stdint.h>

#include "PID.h"

#define MOTOR_0_P 0
#define MOTOR_1_P 0
#define MOTOR_2_P 0
#define MOTOR_3_P 0
#define MOTOR_4_P 0
#define MOTOR_5_P 0
#define MOTOR_6_P 0
#define MOTOR_7_P 0

#define MOTOR_0_I 0
#define MOTOR_1_I 0
#define MOTOR_2_I 0
#define MOTOR_3_I 0
#define MOTOR_4_I 0
#define MOTOR_5_I 0
#define MOTOR_6_I 0
#define MOTOR_7_I 0

#define MOTOR_0_D 0
#define MOTOR_1_D 0
#define MOTOR_2_D 0
#define MOTOR_3_D 0
#define MOTOR_4_D 0
#define MOTOR_5_D 0
#define MOTOR_6_D 0
#define MOTOR_7_D 0

#define MOTOR_0_INT_LIM 0
#define MOTOR_1_INT_LIM 0
#define MOTOR_2_INT_LIM 0
#define MOTOR_3_INT_LIM 0
#define MOTOR_4_INT_LIM 0
#define MOTOR_5_INT_LIM 0
#define MOTOR_6_INT_LIM 0
#define MOTOR_7_INT_LIM 0

#define MOTOR_0_EPSILON 0
#define MOTOR_1_EPSILON 0
#define MOTOR_2_EPSILON 0
#define MOTOR_3_EPSILON 0
#define MOTOR_4_EPSILON 0
#define MOTOR_5_EPSILON 0
#define MOTOR_6_EPSILON 0
#define MOTOR_7_EPSILON 0

#define ZERO_ON_CROSS 0b00000000
#define PID_DELAY 20
#define MAX_MOTOR_SPEED 100

#define SIGN(X) (X == 0 ? 0 : (X > 0 ? 1 : -1))
#define CLAMP(X, Y) (X >= Y ? Y : X)
#define MOTOR_K_ZERO(K) (ZERO_ON_CROSS >> K) & 1

typedef struct pid_controller {
        uint32_t error;
        uint32_t error_sum, last_error;
        uint8_t  output, last_output;
        uint32_t last_time;
        uint16_t dt;
} pid_controller_t;

static uint32_t rpm_targets[8];
static pid_controller_t controllers[8];


static void reset(pid_controller_t *pid)
{
        pid->error       = 0;
        pid->error_sum   = 0;
        pid->last_error  = 0;
        pid->output      = 0;
        pid->last_output = 0;
        pid->last_time   = 0;
        pid->dt          = PID_DELAY; /* Avoid div by 0 */
}

static uint8_t calculate(pid_controller_t *pid, uint16_t error,
                uint8_t motor, uint16_t kP, uint16_t kI, uint16_t kD,
                uint16_t integral_limit, uint16_t epsilon)
{
        /* FIXME Get acual time instead of old RobotC time */
        pid->dt = nPgmTime - pid->last_time;
        pid->last_time = nPgmTime;

        pid->last_error = pid->error;
        pid->error = error;

        float changeInError = pid->dt != 0 ? (pid->error - pid->last_error) / pid->dt : 0;

        if ((MOTOR_K_ZERO(motor)) && (SIGN(pid->error) != SIGN(pid->last_error))) {
                pid->error_sum = 0;
        }

        pid->output = kP * pid->error + kD * changeInError;

        if (abs(pid->output) < MAX_MOTOR_SPEED) {
                pid->error_sum = abs(pid->error) > epsilon ? pid->error_sum + pid->error * pid->dt : 0;
        }

        pid->error_sum = CLAMP(pid->error_sum, integral_limit);
        pid->output += kI * pid->error_sum;

        return pid->output;
}

extern void pid_update()
{
        /* FIXME Figure out how to get real error */
        uint16_t error = 0;

        calculate(&controllers[0], error, 0, MOTOR_0_P, MOTOR_0_I,
                        MOTOR_0_D, MOTOR_0_INT_LIM, MOTOR_0_EPSILON);
        calculate(&controllers[1], error, 1, MOTOR_1_P, MOTOR_1_I,
                        MOTOR_1_D, MOTOR_1_INT_LIM, MOTOR_1_EPSILON);
        calculate(&controllers[2], error, 2, MOTOR_2_P, MOTOR_2_I,
                        MOTOR_2_D, MOTOR_2_INT_LIM, MOTOR_2_EPSILON);
        calculate(&controllers[3], error, 3, MOTOR_3_P, MOTOR_3_I,
                        MOTOR_3_D, MOTOR_3_INT_LIM, MOTOR_3_EPSILON);
        calculate(&controllers[4], error, 4, MOTOR_4_P, MOTOR_4_I,
                        MOTOR_4_D, MOTOR_4_INT_LIM, MOTOR_4_EPSILON);
        calculate(&controllers[5], error, 5, MOTOR_5_P, MOTOR_5_I,
                        MOTOR_5_D, MOTOR_5_INT_LIM, MOTOR_5_EPSILON);
        calculate(&controllers[6], error, 6, MOTOR_6_P, MOTOR_6_I,
                        MOTOR_6_D, MOTOR_6_INT_LIM, MOTOR_6_EPSILON);
        calculate(&controllers[7], error, 7, MOTOR_7_P, MOTOR_7_I,
                        MOTOR_7_D, MOTOR_7_INT_LIM, MOTOR_7_EPSILON);
        calculate(&controllers[7], error, 7, MOTOR_7_P, MOTOR_7_I,
                        MOTOR_7_D, MOTOR_7_INT_LIM, MOTOR_7_EPSILON);
}

extern void update_rpm(motors_t motor_x, uint32_t rpm)
{

}
