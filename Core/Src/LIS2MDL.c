/*
 * LIS2MDL.c
 *
 *  Created on: Sep 3, 2024
 *      Author: liamt
 */

#include "LIS2MDL.h"
#include "cmsis_os.h"
#include "math.h"

// TODO:
//  1. Add more precise error codes instead of blanket LIS2MDL_FAILURE.
//  2. Refine HAL_TIMEOUT (set to ~100 ms right now).

/******************************/
/*           MACROS           */
/******************************/

#define WRITE           (0x00)
#define READ            (0x80)
#define GPIOx           (GPIOB)
#define GPIO_PINx       (GPIO_PIN_4)
#define ENABLE_OFFSET   (0x02)
#define ENABLE_SENSOR   (0x80)
#define SET_SPI_4WIRE   (0x04)
#define REG_WHO_AM_I    (0x4F)
#define REG_CONFIGS     (0x60)
#define REG_OUTPUTS     (0x68)
#define HAL_TIMEOUT     ((uint32_t)1600000)

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static LIS2MDL_STATUS transmit_receive(uint8_t const * const TX,
                                       uint8_t const NUM_TX,
                                       uint8_t * const RX,
                                       uint8_t const NUM_RX,
                                       SPI_HandleTypeDef * const hspi)
{
    LIS2MDL_STATUS status = LIS2MDL_SUCCESS;

    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = LIS2MDL_FAILURE;
    } else {
        if(HAL_SPI_Receive(hspi,
                           (uint8_t *)RX,
                           NUM_RX,
                           HAL_TIMEOUT)
                           != HAL_OK) {
            status = LIS2MDL_FAILURE;
        }
    }
    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_SET);
    taskEXIT_CRITICAL();

    return status;
}

static LIS2MDL_STATUS transmit(uint8_t const * const TX,
                               uint8_t const NUM_TX,
                               SPI_HandleTypeDef * const hspi)
{
    LIS2MDL_STATUS status = LIS2MDL_SUCCESS;

    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = LIS2MDL_FAILURE;
    }
    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_SET);
    taskEXIT_CRITICAL();

    return status;
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

LIS2MDL_STATUS LIS2MDL_Setup(SPI_HandleTypeDef * const hspi)
{
    // Configure LIS2MDL sensor.
    uint8_t const TX1[] = {
        WRITE | REG_CONFIGS,
        WRITE | ENABLE_SENSOR,
        WRITE | ENABLE_OFFSET,
        WRITE | SET_SPI_4WIRE
    };

    if(transmit(TX1, sizeof(TX1) / sizeof(TX1[0]), hspi)
                != LIS2MDL_SUCCESS) {
        return LIS2MDL_FAILURE;
    }

    // If expected WHO_AM_I value is not received, return failure.
    uint8_t const TX2[] = { READ | REG_WHO_AM_I };
    uint8_t RX[1];

    if(transmit_receive(TX2, sizeof(TX2) / sizeof(TX2[0]),
                        RX,  sizeof(RX) / sizeof(RX[0]),
                        hspi) != LIS2MDL_SUCCESS) {
        return LIS2MDL_FAILURE;
    }

    if(RX[0] != 0x40) return LIS2MDL_FAILURE;

    return LIS2MDL_SUCCESS;
}

LIS2MDL_STATUS LIS2MDL_Read(LIS2MDL * const lis2mdl,
                            SPI_HandleTypeDef * const hspi)
{
    uint8_t const TX[] = { READ | REG_OUTPUTS };
    uint8_t RX[6] = {0};

    if(transmit_receive(TX, sizeof(TX) / sizeof(TX[0]),
                        RX, sizeof(RX) / sizeof(RX[0]),
                        hspi) != LIS2MDL_SUCCESS) {
        return LIS2MDL_FAILURE;
    }

    lis2mdl->X = (int16_t)(((uint16_t)RX[1] << 8) | RX[0]);
    lis2mdl->Y = (int16_t)(((uint16_t)RX[3] << 8) | RX[2]);
    lis2mdl->Z = (int16_t)(((uint16_t)RX[5] << 8) | RX[4]);
    lis2mdl->heading = atan2f((float)lis2mdl->Y, (float)lis2mdl->X);

    return LIS2MDL_SUCCESS;
}
