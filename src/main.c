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
#include <stdbool.h>

#define _100_MHZ			(100 * 1000 * 1000)

#define ARRAYSIZE			(10)

#define TESTING_SAMPLES		(2000)

const uint32_t cc1_data[ARRAYSIZE];

uint32_t all_freqs[TESTING_SAMPLES];

bool do_freq_calc = false;

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

	TIM_TimeBaseStructure.TIM_Period = (100 * 1000 * 1000) - 1; // 600 kHz down to 60Hz (2 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1; // 48 MHz Clock down to 600 kHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//Unfortunately the standard peripheral library does not support the max size of timer

	/* TIM IT enable */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	TIM5->DIER |= TIM_IT_CC1 | TIM_IT_CC1DE;

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
	//enable DMA1 Clk
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//create DMA structure
	DMA_InitTypeDef  DMA_InitStructure;
	//reset DMA2 channe1 to default values;
	DMA_DeInit(DMA1_Stream2);
	//channel will be used for memory to memory transfer

	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//setting circular mode
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//medium priority
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//source and destination data size word=32bit
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	//automatic memory destination increment enable.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//source address increment disable
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM5->CCR1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)cc1_data; //[0] ?
	//send values to DMA registers
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	// Enable DMA2 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Stream2, ENABLE); //Enable the DMA2 - Stream 0
}

uint32_t get_hz(){
	//need to stop raw data from updating
	TIM5->DIER &= ~TIM_IT_CC1DE;

	uint8_t k = 0;
	uint8_t dividor = 0;
	uint32_t diff = 0;
	uint32_t temp = 0;
	for(k=0; k < ARRAYSIZE; k++){
		if(cc1_data[(k+1) % ARRAYSIZE] > cc1_data[k]){
			diff = cc1_data[(k+1) % ARRAYSIZE] - cc1_data[k];
			temp += diff;
			dividor++;
		}
	}

	//make sure to run back on the DMA
	TIM5->DIER |= TIM_IT_CC1DE;

	temp = temp / dividor;
	return _100_MHZ / temp;
}

int main(void)
{
	uint32_t d = 0;
	init_LED();
	init_timer();
	init_DMA();
	for(;;){
		if(do_freq_calc == true){
			d = ((d+1) % TESTING_SAMPLES);
			all_freqs[d] = get_hz();
			do_freq_calc = false;
		}
	}
}

void TIM5_IRQHandler(){
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		GPIOD->ODR ^= GPIO_Pin_15;
		do_freq_calc = true;
	}else if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		GPIOD->ODR ^= GPIO_Pin_14;
	}
}

void DMA1_Stream2_IRQHandler(){

}
