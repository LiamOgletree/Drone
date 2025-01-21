/*
 * sensorPA1616S.c
 *
 *  Created on: Oct 26, 2024
 *      Author: liamt
 */

#include "sensorPA1616S.h"

PA1616S_STATUS PA1616S_Setup(UART_HandleTypeDef * const huart,
                             uint8_t * const buf)
{
    // Setup UART for PA1616S to interrupt on idle.
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, PA1616S_BUF_SIZE)
            != HAL_OK) {
        return PA1616S_FAILURE;
    }

    return PA1616S_SUCCESS;
}
