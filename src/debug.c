/************************************************************************
  * @file    debug.c
  * @author  Poorna Kalidas
  * @version V1.0.0
  * @date    28-June-2018
  * @brief   This file provides the debug_write function
  * 
  *  @verbatim
  *  @endverbatim
  ***********************************************************************/
 
 /************************************************************************
 * Includes
 ************************************************************************/
#include "simple_UART.h"

 /************************************************************************
 * Functions
 ************************************************************************/
extern void debug_write(char *message){
#if DEBUG
#warning "DEBUG OUTPUT ENABLED"
	UART_push_out(message);
#endif
}
