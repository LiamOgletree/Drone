/*
 * taskUART.h
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#ifndef INC_TASKUART_H_
#define INC_TASKUART_H_

typedef enum {
    UART_SUCCESS,
    UART_FAILURE
} UART_STATUS;

void StartUART(void *argument);

#endif /* INC_TASKUART_H_ */
