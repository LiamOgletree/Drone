/*
 * LSM6DSO32.c
 *
 *  Created on: Sep 6, 2024
 *      Author: liamt
 */

#include "LSM6DSO32.h"
#include "cmsis_os.h"
#include "sensor.h"

// TODO:
//  1. Add more precise error codes instead of blanket LSM6DSO32_FAILURE.
//  2. Refine HAL_TIMEOUT (set to ~100 ms right now).
//  3. Incorporate UseOffset into existing Read functions.

/******************************/
/*           MACROS           */
/******************************/

#define WRITE                           (0x00)
#define READ                            (0x80)
#define GPIOx                           (GPIOA)
#define GPIO_PINx                       (GPIO_PIN_8)
#define REG_WHO_AM_I                    (0x0F)
#define REG_CTRLS                       (0x10)
#define REG_GYROSCOPE                   (0x22)
#define REG_ACCELEROMETER               (0x28)
#define ENABLE_ACCELEROMETER_LPFILTER   (0x02)
#define ENABLE_GYROSCOPE_HPFILTER       (0x40)
#define DISABLE_I2C                     (0x40)
#define SET_GYROSCOPE_FULLSCALE         (0x02)
#define SET_ODR_104HZ                   (0x40)
#define SET_REG_CTRL3_DEFAULT           (0x04)
#define SET_REG_CTRL4_DEFAULT           (0x00)
#define SET_REG_CTRL5_DEFAULT           (0x00)
#define SET_REG_CTRL6_DEFAULT           (0x00)
#define HAL_TIMEOUT                     ((uint32_t)1600000)


/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static LSM6DSO32_STATUS transmit_receive(uint8_t const * const TX,
                                         uint8_t const NUM_TX,
                                         uint8_t * const RX,
                                         uint8_t const NUM_RX,
                                         SPI_HandleTypeDef * const hspi)
{
    LSM6DSO32_STATUS status = LSM6DSO32_SUCCESS;

    taskENTER_CRITICAL();
    GPIOx->BSRR = (uint32_t)GPIO_PINx << 16U;
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = LSM6DSO32_FAILURE;
    } else {
        if(HAL_SPI_Receive(hspi,
                           (uint8_t *)RX,
                           NUM_RX,
                           HAL_TIMEOUT)
                           != HAL_OK) {
            status = LSM6DSO32_FAILURE;
        }
    }
    GPIOx->BSRR = GPIO_PINx;
    taskEXIT_CRITICAL();

    return status;
}

static LSM6DSO32_STATUS transmit(uint8_t const * const TX,
                                 uint8_t const NUM_TX,
                                 SPI_HandleTypeDef * const hspi)
{
    LSM6DSO32_STATUS status = LSM6DSO32_SUCCESS;

    taskENTER_CRITICAL();
    GPIOx->BSRR = (uint32_t)GPIO_PINx << 16U;
    if(HAL_SPI_Transmit(hspi,
                        (uint8_t *)TX,
                        NUM_TX,
                        HAL_TIMEOUT)
                        != HAL_OK) {
        status = LSM6DSO32_FAILURE;
    }
    GPIOx->BSRR = GPIO_PINx;
    taskEXIT_CRITICAL();

    return status;
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

LSM6DSO32_STATUS LSM6DSO32_Setup(SPI_HandleTypeDef * const hspi)
{
    // If expected WHO_AM_I value is not read, return failure
    uint8_t const TX1[] = { READ | REG_WHO_AM_I };
    uint8_t RX[1] = {0};

    if(transmit_receive(TX1, sizeof(TX1) / sizeof(TX1[0]),
                        RX,  sizeof(RX)  / sizeof(RX[0]),
                        hspi) != LSM6DSO32_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    if(RX[0] != 0x6C) return LSM6DSO32_FAILURE;

    // Otherwise, configure the sensor
    uint8_t const TX2[] = {
        WRITE | REG_CTRLS,
        WRITE | SET_ODR_104HZ | ENABLE_ACCELEROMETER_LPFILTER, // REG CTRL 1 XL
        WRITE | SET_ODR_104HZ | SET_GYROSCOPE_FULLSCALE,       // REG CTRL 2 G
        WRITE | SET_REG_CTRL3_DEFAULT,                         // REG CTRL 3 C
        WRITE | SET_REG_CTRL4_DEFAULT,                         // REG CTRL 4 C
        WRITE | SET_REG_CTRL5_DEFAULT,                         // REG CTRL 5 C
        WRITE | SET_REG_CTRL6_DEFAULT,                         // REG CTRL 6 C
        WRITE | ENABLE_GYROSCOPE_HPFILTER                      // REG CTRL 7 G
    };

    if(transmit(TX2, sizeof(TX2) / sizeof(TX2[0]), hspi)
                != LSM6DSO32_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    return LSM6DSO32_SUCCESS;
}

LSM6DSO32_STATUS LSM6DSO32_ReadGyro(LSM6DSO32 * const lsm6dso32_gyro,
                                    SPI_HandleTypeDef * const hspi,
                                    LSM6DSO32_DATA_TYPE const data_type)
{
    const uint8_t TX[] = { READ  | REG_GYROSCOPE };
    uint8_t RX[6] = {0};

    if(transmit_receive(TX, sizeof(TX) / sizeof(TX[0]),
                        RX, sizeof(RX) / sizeof(RX[0]),
                        hspi) != LSM6DSO32_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    lsm6dso32_gyro->timestamp = TIM5->CNT;

    float sensitivity;
    switch(data_type) {
    case LSM6DSO32_RAW:
        sensitivity = 1.f;
        break;
    case LSM6DSO32_COMPENSATED:
        sensitivity = 4.375f;
        break;
    default:
        return LSM6DSO32_FAILURE;
    }

    int16_t temporary;
    temporary = (int16_t)RX[1] * 256 + (int16_t)RX[0];
    lsm6dso32_gyro->X = (float)temporary * sensitivity;
    temporary = (int16_t)RX[3] * 256 + (int16_t)RX[2];
    lsm6dso32_gyro->Y = (float)temporary * sensitivity;
    temporary = (int16_t)RX[5] * 256 + (int16_t)RX[4];
    lsm6dso32_gyro->Z = (float)temporary * sensitivity;

    return LSM6DSO32_SUCCESS;
}

LSM6DSO32_STATUS LSM6DSO32_ReadAccel(LSM6DSO32 * const lsm6dso32_accel,
                                     SPI_HandleTypeDef * const hspi,
                                     LSM6DSO32_DATA_TYPE const data_type)
{
    uint8_t const TX[] = { READ  | REG_ACCELEROMETER };
    uint8_t RX[6] = {0};

    if(transmit_receive(TX, sizeof(TX) / sizeof(TX[0]),
                        RX, sizeof(RX) / sizeof(RX[0]),
                        hspi) != LSM6DSO32_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    lsm6dso32_accel->timestamp = TIM5->CNT;

    float sensitivity;
    switch(data_type) {
    case LSM6DSO32_RAW:
        sensitivity = 1.f;
        break;
    case LSM6DSO32_COMPENSATED:
        sensitivity = 0.122f;
        break;
    default:
        return LSM6DSO32_FAILURE;
    }

    int16_t temporary;
    temporary = (int16_t)RX[1] * 256 + (int16_t)RX[0];
    lsm6dso32_accel->X = (float)temporary * sensitivity;
    temporary = (int16_t)RX[3] * 256 + (int16_t)RX[2];
    lsm6dso32_accel->Y = (float)temporary * sensitivity;
    temporary = (int16_t)RX[5] * 256 + (int16_t)RX[4];
    lsm6dso32_accel->Z = (float)temporary * sensitivity;

    return LSM6DSO32_SUCCESS;
}

LSM6DSO32_STATUS LSM6DSO32_CalibrateOffset(LSM6DSO32 * const lsm6dso32,
                                           LSM6DSO32_OFFSET * const offset,
                                           SPI_HandleTypeDef * const hspi,
                                           LSM6DSO32_DATA_TYPE const data_type)
{
    LSM6DSO32_STATUS (*readSensor)(LSM6DSO32 * const,
                                   SPI_HandleTypeDef * const,
                                   LSM6DSO32_DATA_TYPE const);

    if(lsm6dso32->sensor == LSM6DSO32_GYROSCOPE) {
        readSensor = LSM6DSO32_ReadGyro;
    } else {
        readSensor = LSM6DSO32_ReadAccel;
    }

    float X, Y, Z;
    for(int i = 0; i < 100; i++) {
        if(readSensor(lsm6dso32, hspi, data_type) != LSM6DSO32_SUCCESS) {
            return LSM6DSO32_FAILURE;
        }

        X += lsm6dso32->X;
        Y += lsm6dso32->Y;
        Z += lsm6dso32->Z;
    }

    offset->X = X / 100.f;
    offset->Y = Y / 100.f;
    offset->Z = Z / 100.f;

    return LSM6DSO32_SUCCESS;
}

LSM6DSO32_STATUS LSM6DSO32_UseOffset(LSM6DSO32 * const lsm6dso32,
                                     LSM6DSO32_OFFSET * const offset)
{
    lsm6dso32->X -= offset->X;
    lsm6dso32->Y -= offset->Y;
    lsm6dso32->Z -= offset->Z;

    return LSM6DSO32_SUCCESS;
}
