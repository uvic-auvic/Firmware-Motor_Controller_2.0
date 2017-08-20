/*
 * simple_UART.h
 *
 *  Created on: Aug 19, 2017
 *      Author: abates
 */

#ifndef SIMPLE_UART_H_
#define SIMPLE_UART_H_

#include "stm32f4xx.h"

#define MAX_OUPUT_DATA (10)

extern void UART_init();
extern void UART_push_out_len(char* mesg, int len);

#endif /* SIMPLE_UART_H_ */
