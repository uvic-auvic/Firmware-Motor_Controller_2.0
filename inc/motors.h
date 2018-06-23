#ifndef MOTORS_H_
#define MOTORS_H_

typedef enum motors {
	Motor1 = 1,
	Motor2,
	Motor3,
	Motor4,
	Motor5,
	Motor6,
	Motor7,
	Motor8
} motors_t;

typedef enum direction {
	Forward = 'F',
	Reverse = 'R'
} direction_t;

extern void init_motors();

extern void motor_set_speed_percent(motors_t motor_x, uint16_t speed, direction_t dir);

extern int16_t motor_get_rpm(motors_t motor_x);

extern void stop_all_motors();

#endif
