#include "stm32f4xx.h"
#include "motors.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define _100_MHZ	(100 * 1000 * 1000)
#define _10_MHZ		(10 * 1000 * 1000)

//PWM In Defines
#define TIM5_PRESCALER	0
#define PWM_IN_TIMER_FREQ	(_100_MHZ /(TIM5_PRESCALER + 1))
#define NUMBER_OF_MOTORS	8
#define PWM_IN_ARRAY_LENGTH 10
#define PULSE_PER_ROTATION	7
#define FREQ_TO_RPM_CONV	((float)60 / (PULSE_PER_ROTATION))
#define INTERNAL_OSC_CALIB	1 // In case we decide to adjust for the manufacturing error in the internal clock
#define MIN_RPM_PERIOD_MS	300 /* Equivalent to 200 RPM */

//PWM Out Defines
#define NEUTRAL				(3600)
#define PWM_OUT_PRESCALER	(42 - 1)
#define PWM_OUT_PERIOD		(40000 - 1)

#if ((PWM_IN_ARRAY_LENGTH %2) == 1)
#error PWM_IN_ARRAY_LENGTH must be a multiple of 2
#endif


/* Global Variables
 * ----------------------------------------------------------
 */
/* Holds the timestamp from TIMER 5 when a rising edge is detected on the 8 input pins for motor rpm.*/
uint32_t pwmInTimestamp[NUMBER_OF_MOTORS][PWM_IN_ARRAY_LENGTH];

/* Holds the direction of the motor. 0 = Forward, 0 = Reverse.
 * This will be updated when the PWM out for the motor is set */
uint8_t motorDirection = 0b00000000;


/* Initialization functions
 * ----------------------------------------------------------
 */

static void init_motor_pwm_out()
{
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//init timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//GPIOA Configuration for motors 1-4 and 7
	GPIO_InitTypeDef GPIOA_InitStruct;
	GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_6;
	GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOA_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOA_InitStruct);

	//GPIOD configuration for motors 5-6
	GPIO_InitTypeDef GPIOD_InitStruct;
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12;
	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);

	//GPIOB configuration for motor 8
	GPIO_InitTypeDef GPIOB_InitStruct;
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);

	//pin alternate function timer configuration
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

	//timer set up
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = PWM_OUT_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_OUT_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//pwm init
	TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //pwm config for channel 1 PA0 motor 1
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    //pwm config for channel 4 PA3 motor 2
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    //pwm config for channel 4 PD15 motor 5
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    //pwm config for channel 1 PD12 motor 6
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    //pwm config for channel 1 PA6 motor 7
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    //pwm config for channel 3 PB0 motor 8
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = NEUTRAL;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	//pwm config for channel 3 PA10 motor 3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	//pwm config for channel 1 PA8 motor 4
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);


    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

static void init_PWM_in_timer() {

	// Enable peripheral clock for TIM1, TIM3, TIM4, TIM5
	RCC->APB1ENR |= RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5;
	RCC->APB2ENR |= RCC_APB2Periph_TIM1;

	/* Time base configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = TIM5_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// Timer Configuration: Motor 1, PA1, TIM5_CH2, DMA1 Stream4 Channel6
	TIM5->DIER |= TIM_IT_CC2DE; //Capture/Compare 1 DMA request enable
	TIM5->CCMR1 |= TIM_CCMR1_CC2S_0; // Configure channel 1 as input
	TIM5->CCER |= TIM_CCER_CC2E; // Enable capture

	// Timer Configuration: Motor 2, PA2, TIM5_CH3, DMA1 Stream0 Channel6
	TIM5->DIER |= TIM_IT_CC3DE;
	TIM5->CCMR2 |= TIM_CCMR2_CC3S_0;
	TIM5->CCER |= TIM_CCER_CC3E;

	// Timer Configuration: Motor 3, PA11, TIM1_CH4, DMA2 Stream4 Channel6
	TIM1->DIER |= TIM_IT_CC4DE;
	TIM1->CCMR2 |= TIM_CCMR2_CC4S_0;
	TIM1->CCER |= TIM_CCER_CC4E;

	// Timer Configuration: Motor 4, PA9, TIM1_CH2, DMA2 Stream2 Channel6
	TIM1->DIER |= TIM_IT_CC2DE;
	TIM1->CCMR1 |= TIM_CCMR1_CC2S_0;
	TIM1->CCER |= TIM_CCER_CC2E;

	// Timer Configuration: Motor 5, PD14, TIM4_CH3, DMA1 Stream7 Channel2
	TIM4->DIER |= TIM_IT_CC3DE;
	TIM4->CCMR2 |= TIM_CCMR2_CC3S_0;
	TIM4->CCER |= TIM_CCER_CC3E;

	// Timer Configuration: Motor 6, PD13, TIM4_CH2, DMA1 Stream3 Channel2
	TIM4->DIER |= TIM_IT_CC2DE;
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;
	TIM4->CCER |= TIM_CCER_CC2E;

	// Timer Configuration: Motor 7, PA7, TIM3_CH2, DMA1 Stream5 Channel5
	TIM3->DIER |= TIM_IT_CC2DE;
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;
	TIM3->CCER |= TIM_CCER_CC2E;

	// Timer Configuration: Motor 8, PB1, TIM3_CH4, DMA1 Stream2 Channel5
	TIM3->DIER |= TIM_IT_CC4DE;
	TIM3->CCMR2 |= TIM_CCMR2_CC4S_0;
	TIM3->CCER |= TIM_CCER_CC4E;

	//Timer will be enabled in another function which will be called last. This is to avoid conflicts.
}


static void init_PWM_in_GPIO() {

	// GPIOA clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Common Configuration for all PWM IN pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	// GPIO Configuration: Motor 1, PA1, TIM5_CH2, DMA1 Stream4 Channel6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); // Connect TIM5 pins to AF

	// GPIO Configuration: Motor 2, PA2, TIM5_CH3, DMA1 Stream0 Channel6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5); // Connect TIM5 pins to AF

	// GPIO Configuration: Motor 3, PA11, TIM1_CH4, DMA2 Stream4 Channel6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1); // Connect TIM1 pins to AF

	// GPIO Configuration: Motor 4, PA9, TIM1_CH2, DMA2 Stream2 Channel6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1); // Connect TIM1 pins to AF

	// GPIO Configuration: Motor 5, PD14, TIM4_CH3, DMA1 Stream7 Channel2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // Connect TIM4 pins to AF

	// GPIO Configuration: Motor 6, PD13, TIM4_CH2, DMA1 Stream3 Channel2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); // Connect TIM4 pins to AF

	// GPIO Configuration: Motor 7, PA7, TIM3_CH2, DMA1 Stream5 Channel5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); // Connect TIM3 pins to AF

	// GPIO Configuration: Motor 8, PB1, TIM3_CH4, DMA1 Stream2 Channel5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); // Connect TIM3 pins to AF

}

static void init_PWM_in_DMA() {

	//enable DMA1 and DMA2 Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
	//create DMA structure
	DMA_InitTypeDef DMA_InitStructure;

	// Common configuration for all DMA streams
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
	DMA_InitStructure.DMA_BufferSize = PWM_IN_ARRAY_LENGTH;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &TIM5->CNT;

	// DMA Configuration: Motor 1, PA1, TIM5_CH2, DMA1 Stream4 Channel6
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pwmInTimestamp[Motor1 - 1];
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE); //Enable the DMA1 - Stream 4

	// DMA Configuration: Motor 2, PA2, TIM5_CH3, DMA1 Stream0 Channel6
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor2 - 1];
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream0, ENABLE); //Enable the DMA1 - Stream 0

	// DMA Configuration: Motor 3, PA11, TIM1_CH4, DMA2 Stream4 Channel6
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor3 - 1];
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4

	// DMA Configuration: Motor 4, PA9, TIM1_CH2, DMA2 Stream2 Channel6
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor4 - 1];
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE); //Enable the DMA2 - Stream 2

	// DMA Configuration: Motor 5, PD14, TIM4_CH3, DMA1 Stream7 Channel2
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor5 - 1];
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream7, ENABLE); //Enable the DMA1 - Stream 7

	// DMA Configuration: Motor 6, PD13, TIM4_CH2, DMA1 Stream3 Channel2
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor6 - 1];
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream3, ENABLE); //Enable the DMA1 - Stream 3

	// DMA Configuration: Motor 7, PA7, TIM3_CH2, DMA1 Stream5 Channel5
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor7 - 1];
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE); //Enable the DMA1 - Stream 5

	// DMA Configuration: Motor 8, PB1, TIM3_CH4, DMA1 Stream2 Channel5
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_Memory0BaseAddr =	(uint32_t) pwmInTimestamp[Motor8 - 1];
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2, ENABLE); //Enable the DMA1 - Stream 2

}

static void init_motor_speed_feedback() {

	init_PWM_in_timer();
	init_PWM_in_GPIO();
	init_PWM_in_DMA();

}

/* Enable Timer 1, 2, 3, 4, 5 */
static void enable_timers() {

	TIM1->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM5->CR1 |= TIM_CR1_CEN;
}

extern void stop_all_motors(){
	motor_set_speed_percent(Motor1, 0, Forward); //PA1
	motor_set_speed_percent(Motor2, 0, Forward); //PA3
	motor_set_speed_percent(Motor3, 0, Forward); //PA10
	motor_set_speed_percent(Motor4, 0, Forward); //PA8
	motor_set_speed_percent(Motor5, 0, Forward); //PD14
	motor_set_speed_percent(Motor6, 0, Forward); //PD12
	motor_set_speed_percent(Motor7, 0, Forward); //PA6
	motor_set_speed_percent(Motor8, 0, Forward); //PB0
}

/* This task is used to check to see if a motor has stopped spinning
 * If a motor stops spinning, then the timestamps won't be updated for an
 * extended period of time. This task checks to see if that condition occurs, and
 * ensures the next RPM reading will be 0 RPM
 */
void rpm_monitor_task(void* dummy) {
	/*
	 * Below is the motor to DMA mapping for RPMs
	 * Motor 1 - DMA1 Stream4 CH6
	 * Motor 2 - DMA1 Stream0 CH6
	 * Motor 3 - DMA2 Stream4 CH6
	 * Motor 4 - DMA2 Stream2 CH6
	 * Motor 5 - DMA1 Stream7 CH2
	 * Motor 6 - DMA1 Stream3 CH2
	 * Motor 7 - DMA1 Stream5 CH5
	 * Motor 8 - DMA1 Stream2 CH5
	 */
	volatile uint32_t *motorToDMA_NTDR[NUMBER_OF_MOTORS] = {
		&DMA1_Stream4->NDTR,
		&DMA1_Stream0->NDTR,
		&DMA2_Stream4->NDTR,
		&DMA2_Stream2->NDTR,
		&DMA1_Stream7->NDTR,
		&DMA1_Stream3->NDTR,
		&DMA1_Stream5->NDTR,
		&DMA1_Stream2->NDTR
	};

	uint8_t motor_indices[NUMBER_OF_MOTORS];
	uint32_t motor_timestamps[NUMBER_OF_MOTORS];
	TickType_t previous_time = 0;

	while(1) {
		/* Update the motor indices and timestamps */
		for(int i = 0; i < NUMBER_OF_MOTORS; i++) {
			motor_indices[i] = *motorToDMA_NTDR[i];
			motor_timestamps[i] = pwmInTimestamp[i][motor_indices[i]];
		}

		/* Wait for the longest period before the motors should turn off */
		vTaskDelayUntil(&previous_time, MIN_RPM_PERIOD_MS);

		/* Check to see if there are any motors that didn't have an updated timestamp */
		for(int i = 0; i < NUMBER_OF_MOTORS; i++) {
			/* Check to see if index is the same */
			if(motor_indices[i] == *motorToDMA_NTDR[i]) {
				/* To make check better, check to see if timestamp is the same */
				if(motor_timestamps[i] == pwmInTimestamp[i][motor_indices[i]]) {
					/* Set all entries of the buffer to last time recorded */
					for(int j = 0; j < PWM_IN_ARRAY_LENGTH; j++) {
						pwmInTimestamp[i][j] = motor_timestamps[i];
					}
				}
			}
		}
	}
}

/* Extern declarations in each section must go below static declarations */
extern void init_motors() {
	/* Initialize sub-modules */
	init_motor_pwm_out();

	init_motor_speed_feedback();

	enable_timers();

	stop_all_motors();

	// Create the RPM monitor task
	xTaskCreate(rpm_monitor_task,
		(const char *)"rpm_monitor_task",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */
}

/* Robert's section.
 * Setting motor speed.
 * ----------------------------------------------------------
 */

/* Put any static/hidden functions your code may need below here: */

/* Robert's extern/visible functions go below here: */
extern void motor_set_speed_percent(motors_t motor_x, uint16_t speed, direction_t dir)
{
	uint16_t cc_value = (speed * 12)/10;
	if(dir == Forward){
		cc_value = NEUTRAL + cc_value;
	}else{
		cc_value = NEUTRAL - cc_value;
	}

	switch(motor_x){
		case Motor1:
			TIM_SetCompare1(TIM2, cc_value);
			break;
		case Motor2:
			TIM_SetCompare4(TIM2, cc_value);
			break;
		case Motor3:
			TIM_SetCompare3(TIM1, cc_value);
			break;
		case Motor4:
			TIM_SetCompare1(TIM1, cc_value);
			break;
		case Motor5:
			TIM_SetCompare4(TIM4, cc_value);
			break;
		case Motor6:
			TIM_SetCompare1(TIM4, cc_value);
			break;
		case Motor7:
			TIM_SetCompare1(TIM3, cc_value);
			break;
		case Motor8:
			TIM_SetCompare3(TIM3, cc_value);
			break;
	}

}

/**
 * Sets the PWM out to the desired value
 * param: percent: PWM duty cycle in 10^-1 percent
 */
extern void set_PWM(motors_t motor_x, uint16_t percent) {

	uint32_t cc_value = ((percent * (PWM_OUT_PERIOD + 1)) / 1000);

	switch(motor_x){
		case Motor1:
			TIM_SetCompare1(TIM2, cc_value);
			break;
		case Motor2:
			TIM_SetCompare4(TIM2, cc_value);
			break;
		case Motor3:
			TIM_SetCompare3(TIM1, cc_value);
			break;
		case Motor4:
			TIM_SetCompare1(TIM1, cc_value);
			break;
		case Motor5:
			TIM_SetCompare4(TIM4, cc_value);
			break;
		case Motor6:
			TIM_SetCompare1(TIM4, cc_value);
			break;
		case Motor7:
			TIM_SetCompare1(TIM3, cc_value);
			break;
		case Motor8:
			TIM_SetCompare3(TIM3, cc_value);
			break;
	}
}

/* Poorna's section.
 * Reading RPM.
 * ----------------------------------------------------------
 */

/* Poorna's extern/visible functions go below here: */

/*
 * Returns motor RPM
 * Param: Motor number
 * Returns: Motor RPM, signed 16-bit int, positive value = forward rotation, negative value = reverse rotation
 *          Max RPM value = 32767. If actual rpm exceeds this, this value will overflow.
 */
extern int16_t motor_get_rpm(motors_t motor_x) {
	/*
	 * Below is the motor to Timer mapping for RPM
	 * Motor 1 - TIM5 CH2
	 * Motor 2 - TIM5 CH3
	 * Motor 3 - TIM1 CH4
	 * Motor 4 - TIM1 CH2
	 * Motor 5 - TIM4 CH3
	 * Motor 6 - TIM4 CH2
	 * Motor 7 - TIM3 CH2
	 * Motor 8 - TIM3 CH4
	 */
	static volatile uint16_t *motorToTIM_DIER[NUMBER_OF_MOTORS] = { &TIM5->DIER,
														   &TIM5->DIER,
														   &TIM1->DIER,
														   &TIM1->DIER,
														   &TIM4->DIER,
														   &TIM4->DIER,
														   &TIM3->DIER,
														   &TIM3->DIER };

	static uint16_t motorToTIM_CCxDE[NUMBER_OF_MOTORS] = { TIM_IT_CC2DE,
														   TIM_IT_CC3DE,
														   TIM_IT_CC4DE,
														   TIM_IT_CC2DE,
														   TIM_IT_CC3DE,
														   TIM_IT_CC2DE,
														   TIM_IT_CC2DE,
														   TIM_IT_CC4DE };

	/*
	 * Below is the motor to DMA mapping for RPM
	 * Motor 1 - DMA1 Stream4 CH6
	 * Motor 2 - DMA1 Stream0 CH6
	 * Motor 3 - DMA2 Stream4 CH6
	 * Motor 4 - DMA2 Stream2 CH6
	 * Motor 5 - DMA1 Stream7 CH2
	 * Motor 6 - DMA1 Stream3 CH2
	 * Motor 7 - DMA1 Stream5 CH5
	 * Motor 8 - DMA1 Stream2 CH5
	 */
	static volatile uint32_t *motorToDMA_NTDR[NUMBER_OF_MOTORS] = { &DMA1_Stream4->NDTR,
														   &DMA1_Stream0->NDTR,
														   &DMA2_Stream4->NDTR,
														   &DMA2_Stream2->NDTR,
														   &DMA1_Stream7->NDTR,
														   &DMA1_Stream3->NDTR,
														   &DMA1_Stream5->NDTR,
														   &DMA1_Stream2->NDTR };

	//need to stop raw data from updating
	*motorToTIM_DIER[motor_x - 1] &= ~motorToTIM_CCxDE[motor_x - 1];

	uint8_t idx = PWM_IN_ARRAY_LENGTH - (*motorToDMA_NTDR[motor_x - 1]); //Index of array element that was most recently updated
	uint8_t stop_idx = idx;
	float frequency = 0;
	int16_t rpm = 0;

	// Average the time differences
	do {
		uint32_t diff = 0; // Temporarily store the difference between two timestamps

		if (pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH] >= pwmInTimestamp[motor_x - 1][idx]) {
			diff = pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH]	- pwmInTimestamp[motor_x - 1][idx];

		} else { //if not bigger, must be smaller
			diff = (_100_MHZ - pwmInTimestamp[motor_x - 1][idx]) + pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH];
		}
		frequency += diff; // Add the differences. Will be averaged later

		/* Update the index */
		idx = ((idx + 2) % PWM_IN_ARRAY_LENGTH);
	} while(idx != stop_idx);

	//make sure to turn the DMA back on
	*motorToTIM_DIER[motor_x - 1] |= motorToTIM_CCxDE[motor_x - 1];

	// If frequency (which is actually period) is 0, then let RPM stay at zero
	if(frequency) {
		frequency = frequency / (PWM_IN_ARRAY_LENGTH / 2); // Average the timestamps
		frequency = PWM_IN_TIMER_FREQ / frequency; // Convert timestamps to frequency
		rpm = frequency * FREQ_TO_RPM_CONV * INTERNAL_OSC_CALIB; //Convert frequency to rpm and apply correction for internal oscillator
		// Not checking for overflow. Motor RPM will likely never reach the 32767rpm.

		if ((motorDirection >> (motor_x - 1)) & 0x1) { // If bit (motor_x - 2) is 0, motor is spinning forward, reverse otherwise
			rpm *= -1;
		}
	}

	return rpm;
}
