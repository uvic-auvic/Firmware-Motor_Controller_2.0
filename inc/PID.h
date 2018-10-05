#ifndef PID_H_
#define PID_H_

typedef struct PID_controller {
    uint16_t P, I, D;
    uint32_t error;
    uint32_t errorSum, lastError;
    uint32_t output, lastOutput;
    uint32_t lastTime;
    uint16_t integralLimit, epsilon;
    uint16_t dTime;
} PID_controller_t;

/**
 * Initializes the PID controller with the
 * provided values. There are many
 *
 * @param pid           The PID controller to initialize
 * @param kP            The P value for the controller
 * @param kI            The I value for the controller
 * @param kD            The D value for the controller
 * @param integralLimit The integral limit
 * @param epsilon       The epsilon value
 * @param slewRate      The slew rate
 * @param zeroOnCross   Reset integral term on zero cross
 * @param refreshRate   The delay between loops
 */
extern void PID_init(PID_controller_t *pid, uint32_t kP, uint32_t kI,
        uint32_t kD, uint16_t integral_limit, uint16_t epsilon);

/**
 * Resets all critical values for the PID
 * controller provided
 *
 * @param pid The controller to reset
 */
extern void PID_reset(PID_controller_t *pid);

/**
 * Computes the overall PID output using the
 * provided error
 *
 * @param pid   The PID controller to use
 * @param error Error to use for the calculation
 * @return The output value of the PID controller
 */
extern uint32_t PID_calculate(PID_controller_t *pid, uint32_t error);

#endif
