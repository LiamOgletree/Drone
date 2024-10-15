/*
 * sensorShared.c
 *
 *  Created on: Oct 11, 2024
 *      Author: liamt
 */

#include "sensorShared.h"

// TODO:
//  1. Refine HAL_TIMEOUT (set to ~100 ms right now).

/******************************/
/*           MACROS           */
/******************************/

#define HAL_TIMEOUT ((uint32_t)1600000)

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

SENSOR_STATUS sensor_tx_rx(uint8_t const * const TX,
                           uint8_t const NUM_TX,
                           uint8_t * const RX,
                           uint8_t const NUM_RX,
                           SPI_HandleTypeDef * const hspi,
                           GPIO_TypeDef * const GPIOx,
                           uint16_t const GPIO_PINx)
{
    SENSOR_STATUS status = SENSOR_SUCCESS;

    // Enter a "critical" section to make sure we don't get
    //  preempted in the middle of a SPI transaction.
    taskENTER_CRITICAL();

    // Drive the correct Chip Select low (ready).
    GPIOx->BSRR = (uint32_t)GPIO_PINx << 16U;

    // Perform the SPI transaction.
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = SENSOR_FAILURE;
    } else {
        if(HAL_SPI_Receive(hspi,
                           (uint8_t *)RX,
                           NUM_RX,
                           HAL_TIMEOUT)
                           != HAL_OK) {
            status = SENSOR_FAILURE;
        }
    }

    // Drive the correct Chip Select high again.
    GPIOx->BSRR = GPIO_PINx;

    // Exit the critical section to allow other tasks
    //  to preempt again.
    taskEXIT_CRITICAL();

    return status;
}

__attribute__((optimize("-Ofast")))
SENSOR_STATUS sensor_tx(uint8_t const * TX,
                        uint8_t const NUM_TX,
                        SPI_HandleTypeDef * const hspi,
                        GPIO_TypeDef * const GPIOx,
                        uint16_t const GPIO_PINx)
{
    SENSOR_STATUS status = SENSOR_SUCCESS;

    // Enter a "critical" section to make sure we don't get
    //  preempted in the middle of a SPI transaction.
    taskENTER_CRITICAL();

    // Drive the correct Chip Select low (ready).
    GPIOx->BSRR = (uint32_t)GPIO_PINx << 16U;

    // Perform the SPI transaction.
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = SENSOR_FAILURE;
    }

    // Drive the correct Chip Select high again.
    GPIOx->BSRR = GPIO_PINx;

    // Exit the critical section to allow other tasks
    //  to preempt again.
    taskEXIT_CRITICAL();

    return status;
}
