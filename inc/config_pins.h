/*
 * config_pins.h
 *
 *  Created on: May 19, 2018
 *      Author: auvic
 */

#ifndef CONFIG_PINS_H_
#define CONFIG_PINS_H_

typedef enum configPins {
	configPin_1 = 1,
	configPin_2,
	configPin_3,
	configPin_4,

} configPins_t;

extern void config_pins_init();
extern uint8_t read_config_pin(configPins_t pinRead);

#endif /* CONFIG_PINS_H_ */
