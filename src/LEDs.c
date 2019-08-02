/************************************************************************
  * @file    LEDs.c
  * @author  Andy Bates
  * @version V1.0.0
  * @date    13-August-2017
  * @brief   This file contains the LED and PWM init functions.
  * 
  *  @verbatim
  *  @endverbatim
  */

/* Includes -----------------------------------------------------------*/
#include "stm32f4xx_gpio.h"

/**
  * @user	None
  * @brief  Configure PA4 (Red) and PA5 (Blue) to blinkat 2/50 MHz
  * @param  None
  * @retval None
  */
extern void init_LED(){
	GPIO_InitTypeDef GPIO_InitStructure;

	//Enable the A port
	//Enable the D and A port to be used, D is for break-out board A for PCBA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//For now use the GPIO_Init function, to save time
	//Be careful because not erasing any of the past settings

	//Configure pin PA4(red LED) and pin PA5(blue LED) as output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
