#ifndef PID_H_
#define PID_H_

typedef struct PID_controller {
    uint32_t error;
    uint32_t error_sum, last_error;
    uint32_t output, last_output;
    uint32_t last_time;
    uint16_t dt;
} PID_controller_t;

/* TODO: Don't forget to add delay for overall PID
 * update loop task */

/**
 * Resets all critical values for the PID
 * controller provided
 *
 * @param pid The controller to reset
 */
void PID_reset(PID_controller_t *pid);

/**
 * Computes the overall PID output using the
 * provided error
 *
 * @param pid   The PID controller to use
 * @param error Error to use for the calculation
 * @return The output value of the PID controller (-100 - 100 %)
 */
int8_t PID_calculate(PID_controller_t *pid, uint32_t error);

#endif
