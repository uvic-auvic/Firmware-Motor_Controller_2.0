#include "stm32f4xx.h"
#include "motors.h"

#define _100_MHZ	(100 * 1000 * 1000)
#define _10_MHZ		(10 * 1000 * 1000)

#define TIM5_PRESCALER	0
#define PWM_IN_TIMER_FREQ	(_100_MHZ /(TIM5_PRESCALER + 1))
#define NUMBER_OF_MOTORS	8
#define PWM_IN_ARRAY_LENGTH 10
#define PULSE_PER_ROTATION	7
#define FREQ_TO_RPM_CONV	((float)60 / (PULSE_PER_ROTATION))
#define INTERNAL_OSC_CALIB	1 // In case we decide to adjust for the manufacturing error in the internal clock

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
static void init_motor_current_temp_MUX() {
	//Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//GPIOC Configuration
	GPIO_InitTypeDef GPIOC_InitStruct;
	GPIOC_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIOC_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOC_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOC_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOC_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIOC_InitStruct);

	//GPIOD Configuration
	GPIO_InitTypeDef GPIOD_InitStruct;
	GPIOD_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;

	GPIOD_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOD_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOD_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIOD_InitStruct);
}

static void init_motor_speed_control() {

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

/* Extern declarations in each section must go below static declarations */
extern void init_motors() {
	/* Initialize sub-modules */
	init_motor_current_temp_MUX();
	init_motor_speed_control();
	init_motor_speed_feedback();
	enable_timers();
}

/* Robert's section.
 * Setting motor speed.
 * ----------------------------------------------------------
 */

/* Put any static/hidden functions your code may need below here: */

/* Robert's extern/visible functions go below here: */
extern void motor_set_speed_percent(motors_t motor_x, uint8_t speed, direction_t dir) {

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
	static uint16_t *motorToTIM_DIER[NUMBER_OF_MOTORS] = { &TIM5->DIER,
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
	static uint32_t *motorToDMA_NTDR[NUMBER_OF_MOTORS] = { &DMA1_Stream4->NDTR,
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
	uint8_t stop_idx = (idx + PWM_IN_ARRAY_LENGTH - 1) % PWM_IN_ARRAY_LENGTH; //Index of array element that is most out of date
	float frequency = 0;
	int16_t rpm = 0;

	for (; idx != stop_idx; idx = ((idx + 1) % PWM_IN_ARRAY_LENGTH)) {
		uint32_t diff = 0; // Temporarily store the difference between two timestamps

		if (pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH] > pwmInTimestamp[motor_x - 1][idx]) {
			diff = pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH]	- pwmInTimestamp[motor_x - 1][idx];

		} else { //if not bigger, must be smaller
			diff = (_100_MHZ - pwmInTimestamp[motor_x - 1][idx]) + pwmInTimestamp[motor_x - 1][(idx + 1) % PWM_IN_ARRAY_LENGTH];
		}
		frequency += diff; // Add the differences. Will be averaged later
	}

	//make sure to turn the DMA back on
	*motorToTIM_DIER[motor_x - 1] |= motorToTIM_CCxDE[motor_x - 1];

	frequency = frequency / (PWM_IN_ARRAY_LENGTH - 1); // Average the timestamps
	frequency = PWM_IN_TIMER_FREQ / frequency; // Convert timestamps to frequency
	rpm = frequency * FREQ_TO_RPM_CONV * INTERNAL_OSC_CALIB; //Convert frequency to rpm and apply correction for internal oscillator
	// Not checking for overflow. Motor RPM will likely never reach the 32767rpm.

	if ((motorDirection >> (motor_x - 1)) & 0x1) { // If bit (motor_x - 2) is 0, motor is spinning forward, reverse otherwise
		rpm *= -1;
	}

	return rpm;
}

/* Code from Issue #8 (by Robert Keen).
 * Reading MUX'ed sensor data.
 * ----------------------------------------------------------
 */

extern uint8_t get_motor_current(motor_sensors_t motor_sensor_x) {
	switch (motor_sensor_x) {
		case Motor_Curr_ADC1:
			GPIOC->BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9); //turn off PC7-PC9
			return 0;
			break;
		case Motor_Curr_ADC2:
			GPIOC->BSRRL |= GPIO_Pin_7;	//turn on PC7
			return 0;
			break;
		case Motor_Curr_ADC3:
			GPIOC->BSRRL |= GPIO_Pin_8; //turn on PC8
			return 0;
			break;
		case Motor_Curr_ADC4:
			GPIOC->BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PC7-PC8
			return 0;
			break;
		case Motor_Curr_ADC5:
			GPIOC->BSRRL |= GPIO_Pin_9; //turn on PC9
			return 0;
			break;
		case Motor_Curr_ADC6:
			GPIOC->BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PC7-PC9
			return 0;
			break;
		case Motor_Curr_ADC7:
			GPIOC->BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PC8-PC9
			return 0;
			break;
		case Motor_Curr_ADC8:
			GPIOC->BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PC7-PC9
			return 0;
			break;
		default:
			return 0;
			break;
	}
}

extern uint8_t get_motor_temp(motor_sensors_t motor_sensor_x) {
	switch (motor_sensor_x) {
		case Motor_Temp_ADC1:
			GPIOD->BSRRL &= ~(GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9); //turn off PD7-PD9
			return 0;
			break;
		case Motor_Temp_ADC2:
			GPIOD->BSRRL |= GPIO_Pin_7; //turn on PD7
			return 0;
			break;
		case Motor_Temp_ADC3:
			GPIOD->BSRRL |= GPIO_Pin_8; //turn on PD8
			return 0;
			break;
		case Motor_Temp_ADC4:
			GPIOD->BSRRL |= GPIO_Pin_7 | GPIO_Pin_8; //turn on PD7-PD8
			return 0;
			break;
		case Motor_Temp_ADC5:
			GPIOD->BSRRL |= GPIO_Pin_9; //turn on PD9
			return 0;
			break;
		case Motor_Temp_ADC6:
			GPIOD->BSRRL |= GPIO_Pin_7 | GPIO_Pin_9; //turn on PD7-PD9
			return 0;
			break;
		case Motor_Temp_ADC7:
			GPIOD->BSRRL |= GPIO_Pin_8 | GPIO_Pin_9; //turn on PD8-PD9
			return 0;
			break;
		case Motor_Temp_ADC8:
			GPIOD->BSRRL |= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //turn on PD7-PD9
			return 0;
			break;
		default:
			return 0;
			break;
	}
}
