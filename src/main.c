/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

void init_PWM_GPIO(){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration: TIM5 CH1 (PA0) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM5 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
}

void init_PWM_in(){
	TIM5->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1;

	TIM5->CCER |= TIM_CCER_CC1E;

	//TIM5->CR1 |= TIM_CR1_CEN;

	init_PWM_GPIO();
}

void init_LED(){
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

void init_timer(){
	/*enable peripheral clock for TIM5 */
	RCC->APB1ENR |= 0x8;

	/* Time base configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = 10000 - 1; // 600 kHz down to 60Hz (2 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 10000 - 1; // 48 MHz Clock down to 600 kHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* TIM IT enable */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	TIM5->DIER |= TIM_IT_CC1;

	init_PWM_in();

	/* TIM5 enable counter */
	TIM_Cmd(TIM5, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM1 interrupt from the interrupt table */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void init_DMA(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
}

int main(void)
{
	int d = 0;
	init_LED();
	init_timer();
	init_DMA();
	for(;;){
		d = RCC->AHB1ENR;

	}
}

void TIM5_IRQHandler(){
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		GPIOD->ODR ^= GPIO_Pin_15;
	}else if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		GPIOD->ODR ^= GPIO_Pin_14;
	}
}
