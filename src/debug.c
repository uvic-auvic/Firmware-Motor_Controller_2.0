/*
 * debug.c
 *
 *  Created on: Jun 28, 2018
 *      Author: Poornachander
 */
#include "simple_UART.h"

extern void debug_write(char *message) {
#if DEBUG
#warning "DEBUG OUTPUT ENABLED"
	UART_push_out(message);
#endif
}
