/*
 * config_pins.c
 *
 *  Created on: May 19, 2018
 *      Author: auvic
 */

#include "stm32f4xx.h"
#include "config_pins.h"

uint8_t configpins = 0;

extern void config_pins_init() {

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	configpins |= GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
	configpins |=  (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)) << 1;
	configpins |=  (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13)) << 2;
	configpins |=  (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14)) << 3;

}

extern uint8_t read_config_pin(configPins_t pinRead) {

	uint8_t inputData;
	if ((pinRead == 1)){
		inputData = (configpins & 0x01);
	}
	else if (pinRead == 2){
		inputData = (configpins & 0x02) == 0x02;
	}
	else if(pinRead == 3){
		inputData = (configpins & 0x04) == 0x04;
	}
	else if(pinRead == 4){
		inputData = (configpins & 0x08) == 0x08;
	}

	return inputData;
}
