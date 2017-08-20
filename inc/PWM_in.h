/*
 * PWM_in.h
 *
 *  Created on: Aug 13, 2017
 *      Author: auvic
 */

#ifndef PWM_IN_H_
#define PWM_IN_H_

#define _100_MHZ			(100 * 1000 * 1000)
#define _10_MHZ				(10 * 1000 * 1000)

#define ARRAYSIZE			(10)

extern void init_pwm_in();
extern uint32_t get_rpm();

#endif /* PWM_IN_H_ */
