#include <stdlib.h>
#include <stdint.h>

#include "PID.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

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

static int16_t rpm_targets[8];
static pid_controller_t controllers[8];

static uint8_t calculate(pid_controller_t *pid, uint16_t error,
                uint8_t motor, uint16_t kP, uint16_t kI, uint16_t kD,
                uint16_t integral_limit, uint16_t epsilon)
{
        pid->dt =  xTaskGetTickCount() - pid->last_time;
        pid->last_time = xTaskGetTickCount();

        pid->last_error = pid->error;
        pid->error = error;

        float de = pid->dt != 0 ? (pid->error - pid->last_error) / pid->dt : 0;

        if ((MOTOR_K_ZERO(motor)) && (SIGN(pid->error) != SIGN(pid->last_error))) {
                pid->error_sum = 0;
        }

        pid->output = kP * pid->error + kD * de;

        if (abs(pid->output) < MAX_MOTOR_SPEED) {
                pid->error_sum = abs(pid->error) > epsilon
                        ? pid->error_sum + pid->error * pid->dt : 0;
        }

        pid->error_sum = CLAMP(pid->error_sum, integral_limit);
        pid->output += kI * pid->error_sum;

        /* FIXME: Use abs output for directions to avoid motor breaking*/
        return pid->output;
}

static direction_t get_dir(pid_controller_t *pid)
{
        return pid->output == 0 ? Forward : Reverse;
}

extern void pid_update()
{
        uint16_t error0 = rpm_targets[0] - motor_get_rpm(Motor1);
        uint16_t error1 = rpm_targets[1] - motor_get_rpm(Motor2);
        uint16_t error2 = rpm_targets[2] - motor_get_rpm(Motor3);
        uint16_t error3 = rpm_targets[3] - motor_get_rpm(Motor4);
        uint16_t error4 = rpm_targets[4] - motor_get_rpm(Motor5);
        uint16_t error5 = rpm_targets[5] - motor_get_rpm(Motor6);
        uint16_t error6 = rpm_targets[6] - motor_get_rpm(Motor7);
        uint16_t error7 = rpm_targets[7] - motor_get_rpm(Motor8);

        motor_set_speed_percent(Motor1, calculate(&controllers[0],
                                error0, 0, MOTOR_0_P, MOTOR_0_I,
                                MOTOR_0_D, MOTOR_0_INT_LIM,
                                MOTOR_0_EPSILON), get_dir(&controllers[0]));
        motor_set_speed_percent(Motor2, calculate(&controllers[1],
                                error1, 1, MOTOR_1_P, MOTOR_1_I,
                                MOTOR_1_D, MOTOR_1_INT_LIM,
                                MOTOR_1_EPSILON), get_dir(&controllers[1]));

        motor_set_speed_percent(Motor3, calculate(&controllers[2],
                                error2, 2, MOTOR_2_P, MOTOR_2_I,
                                MOTOR_2_D, MOTOR_2_INT_LIM,
                                MOTOR_2_EPSILON), get_dir(&controllers[2]));

        motor_set_speed_percent(Motor4, calculate(&controllers[3],
                                error3, 3, MOTOR_3_P, MOTOR_3_I,
                                MOTOR_3_D, MOTOR_3_INT_LIM,
                                MOTOR_3_EPSILON), get_dir(&controllers[3]));

        motor_set_speed_percent(Motor5, calculate(&controllers[4],
                                error4, 4, MOTOR_4_P, MOTOR_4_I,
                                MOTOR_4_D, MOTOR_4_INT_LIM,
                                MOTOR_4_EPSILON), get_dir(&controllers[4]));

        motor_set_speed_percent(Motor6, calculate(&controllers[5],
                                error5, 5, MOTOR_5_P, MOTOR_5_I,
                                MOTOR_5_D, MOTOR_5_INT_LIM,
                                MOTOR_5_EPSILON), get_dir(&controllers[5]));

        motor_set_speed_percent(Motor7, calculate(&controllers[6],
                                error6, 6, MOTOR_6_P, MOTOR_6_I,
                                MOTOR_6_D, MOTOR_6_INT_LIM,
                                MOTOR_6_EPSILON), get_dir(&controllers[6]));

        motor_set_speed_percent(Motor8, calculate(&controllers[7],
                                error7, 7, MOTOR_7_P, MOTOR_7_I,
                                MOTOR_7_D, MOTOR_7_INT_LIM,
                                MOTOR_7_EPSILON), get_dir(&controllers[7]));
}

extern void pid_reset(motors_t motor_x)
{
        (&controllers[motor_x - 1])->error       = 0;
        (&controllers[motor_x - 1])->error_sum   = 0;
        (&controllers[motor_x - 1])->last_error  = 0;
        (&controllers[motor_x - 1])->output      = 0;
        (&controllers[motor_x - 1])->last_output = 0;
        (&controllers[motor_x - 1])->last_time   = 0;
        (&controllers[motor_x - 1])->dt          = PID_DELAY; /* Avoid div by 0 */
}

extern void pid_update_rpm(motors_t motor_x, int16_t rpm)
{
        rpm_targets[motor_x] = rpm;
}
