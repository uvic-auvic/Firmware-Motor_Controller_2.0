/*
 * LEDs.c
 *
 *  Created on: Aug 13, 2017
 *      Author: auvic
 */

#include "stm32f4xx_gpio.h"

extern void init_LED(){
	GPIO_InitTypeDef GPIO_InitStructure;

	//Andy's attempt at blinking an LED

	//Enable the D port to be used
	//RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//For now use the GPIO_Init function, to save time
	//Be careful because not erasing any of the past settings so failure to overwrite will keep them

	//Select port 3 to be our toggle port
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
}
