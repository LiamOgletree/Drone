/*
 * taskPA1616S.h
 *
 *  Created on: Oct 26, 2024
 *      Author: liamt
 */

#ifndef INC_TASKPA1616S_H_
#define INC_TASKPA1616S_H_

void StartPA1616S(void *argument);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size);

#endif /* INC_TASKPA1616S_H_ */
