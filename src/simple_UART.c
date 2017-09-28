/*
 * simple_UART.c
 *
 *  Created on: Aug 19, 2017
 *      Author: abates
 */

#include "simple_UART.h"
#include <string.h>

char stringtosend[MAX_OUPUT_DATA] = "";

uint8_t bytes_to_send;

static void Configure_GPIO_USART1(void)
{
	/* Enable the peripheral clock of GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Configuration: TIM5 CH1 (PA0) */
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect USART1 pins to AF */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

/**
  * @brief  This function configures USART1.
  * @param  None
  * @retval None
  */
static void Configure_USART1(void)
{
	/* Enable the peripheral clock USART1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//RCC->CFGR3 |= RCC_CFGR3_USART1SW_1;
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization

	USART_InitStruct.USART_BaudRate = 9600;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
	NVIC_EnableIRQ(USART1_IRQn); /* (4) */

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

extern void UART_push_out_len(char* mesg, int len){
	bytes_to_send = len;
	memcpy(stringtosend, mesg, len);
	//We need to wait for data register to be empty
	len = (USART1->SR & 0x00000040);
	while( !len );
	USART_SendData(USART1, mesg[0]);
}

extern void UART_init(){
	bytes_to_send = 0;

	//initialize the UART driver
	Configure_GPIO_USART1();
	Configure_USART1();
}

void USART1_IRQHandler(){

}
