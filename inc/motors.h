#ifndef MOTORS_H_
#define MOTORS_H_

#define MAX_MOTOR_RPM	(1000)

typedef enum motors {
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_5,
	MOTOR_6,
	MOTOR_7,
	MOTOR_8
} motors_t;

typedef enum direction {
	Forward = 'F',
	Reverse = 'R'
} direction_t;

extern void init_motors();

extern void motor_set_speed_percent(motors_t motor_x, uint16_t speed, direction_t dir);

extern void set_PWM(motors_t motor_x, uint16_t percent);

extern int16_t motor_get_rpm(motors_t motor_x);

extern void stop_all_motors();

#endif
