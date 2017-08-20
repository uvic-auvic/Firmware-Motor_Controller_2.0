//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef UART_Receiver_H_
#define UART_Receiver_H_

#include "stm32f4xx.h"
#include "FSM.h"
#include <stdbool.h>

/* Public function prototypes -----------------------------------------------*/
extern void UART_init();
extern void UART_push_out(char* mesg);
extern void UART_push_out_len(char* mesg, int len);
// ----------------------------------------------------------------------------

// --------------------------------------------------------------------------*/

#endif // UART_Receiver_H_
