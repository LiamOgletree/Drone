/*
 * LSM6DSO32.c
 *
 *  Created on: Sep 6, 2024
 *      Author: liamt
 */

#include "sensorLSM6DSO32.h"
#include "sensorShared.h"
#include "cmsis_os.h"

// TODO:
//  1. Add more precise error codes instead of blanket LSM6DSO32_FAILURE.
//  2. Refine HAL_TIMEOUT (set to ~100 ms right now).
//  3. Incorporate UseOffset into existing Read functions.

/******************************/
/*           MACROS           */
/******************************/

#define WRITE                     (0x00)
#define READ                      (0x80)
#define GPIOx                     (GPIOA)
#define GPIO_PINx                 (GPIO_PIN_8)
#define REG_WHO_AM_I              (0x0F)
#define REG_CTRLS                 (0x10)
#define REG_GYROSCOPE             (0x22)
#define REG_ACCELEROMETER         (0x28)
#define EN_ACCELEROMETER_LPFILTER (0x02)
#define EN_GYROSCOPE_HPFILTER     (0x40)
#define SET_GYROSCOPE_FULLSCALE   (0x02)
#define SET_ODR_104HZ             (0x40)
#define SET_REG_CTRL3_DEFAULT     (0x04)
#define SET_REG_CTRL4_DEFAULT     (0x00)
#define SET_REG_CTRL5_DEFAULT     (0x00)
#define SET_REG_CTRL6_DEFAULT     (0x00)

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

LSM6DSO32_STATUS LSM6DSO32_Setup(SPI_HandleTypeDef * const hspi)
{
    uint8_t const TX1[] = { READ | REG_WHO_AM_I };
    uint8_t RX[1] = {0};

    // Conduct SPI transaction to retrieve WHO_AM_I register value.
    if(sensor_tx_rx(TX1, sizeof(TX1) / sizeof(TX1[0]),
                    RX,  sizeof(RX)  / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    // If expected WHO_AM_I value is not retrieved, return failure.
    if(RX[0] != 0x6C) return LSM6DSO32_FAILURE;

    // Configure the LSM6DSO32 sensor.
    uint8_t const TX2[] = {
        WRITE | REG_CTRLS,
        WRITE | SET_ODR_104HZ | EN_ACCELEROMETER_LPFILTER, // REG CTRL 1 XL
        WRITE | SET_ODR_104HZ | SET_GYROSCOPE_FULLSCALE,   // REG CTRL 2 G
        WRITE | SET_REG_CTRL3_DEFAULT,                     // REG CTRL 3 C
        WRITE | SET_REG_CTRL4_DEFAULT,                     // REG CTRL 4 C
        WRITE | SET_REG_CTRL5_DEFAULT,                     // REG CTRL 5 C
        WRITE | SET_REG_CTRL6_DEFAULT,                     // REG CTRL 6 C
        WRITE | EN_GYROSCOPE_HPFILTER                      // REG CTRL 7 G
    };

    // Conduct SPI transaction to setup the LSM6DSO32 sensor.
    if(sensor_tx(TX2, sizeof(TX2) / sizeof(TX2[0]),
                 hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
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

    // Conduct SPI transaction to read the gyroscope data
    //  output registers.
    if(sensor_tx_rx(TX, sizeof(TX) / sizeof(TX[0]),
                    RX, sizeof(RX) / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    // Save time stamp to compute time steps for state machine.
    lsm6dso32_gyro->timestamp = TIM5->CNT;

    // Set scaling factor if compensating the raw readings.
    float const sensitivity = (data_type == LSM6DSO32_RAW) ? 1.f : 4.375f;

    // Translate raw gyroscope readings into degrees per second floats.
    int16_t temporary = (int16_t)RX[1] * 256 + (int16_t)RX[0];
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

    // Conduct SPI transaction to read the accelerometer data
    //  output registers.
    if(sensor_tx_rx(TX, sizeof(TX) / sizeof(TX[0]),
                    RX, sizeof(RX) / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return LSM6DSO32_FAILURE;
    }

    // Save time stamp to compute time steps for state machine.
    lsm6dso32_accel->timestamp = TIM5->CNT;

    // Set scaling factor if compensating the raw readings.
    float const sensitivity = (data_type == LSM6DSO32_RAW) ? 1.f : 0.122f;

    // Translate raw accelerometer readings into meters per second^2 floats.
    int16_t temporary = (int16_t)RX[1] * 256 + (int16_t)RX[0];
    lsm6dso32_accel->X = (float)temporary * sensitivity;
    temporary = (int16_t)RX[3] * 256 + (int16_t)RX[2];
    lsm6dso32_accel->Y = (float)temporary * sensitivity;
    temporary = (int16_t)RX[5] * 256 + (int16_t)RX[4];
    lsm6dso32_accel->Z = (float)temporary * sensitivity;

    return LSM6DSO32_SUCCESS;
}
