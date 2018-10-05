#ifndef PID_H_
#define PID_H_

typedef struct PID_controller {
    uint16_t P, I, D;
    uint32_t error;
    uint32_t error_sum, last_error;
    uint32_t output, last_output;
    uint32_t last_time;
    uint16_t integral_limit, epsilon;
    uint16_t dt;
} PID_controller_t;

/* TODO: Add delay for overall PID update loop task */

/**
 * Initializes the PID controller with the
 * provided values. There are many
 *
 * @param pid            The PID controller to initialize
 * @param kP             The P value for the controller
 * @param kI             The I value for the controller
 * @param kD             The D value for the controller
 * @param integral_limit The integral limit
 * @param epsilon        The epsilon value
 */
static void PID_init(PID_controller_t *pid, uint32_t kP, uint32_t kI,
        uint32_t kD, uint16_t integral_limit, uint16_t epsilon);

/**
 * Resets all critical values for the PID
 * controller provided
 *
 * @param pid The controller to reset
 */
static void PID_reset(PID_controller_t *pid);

/**
 * Computes the overall PID output using the
 * provided error
 *
 * @param pid   The PID controller to use
 * @param error Error to use for the calculation
 * @return The output value of the PID controller
 */
static uint32_t PID_calculate(PID_controller_t *pid, uint32_t error);

#endif
