/*
 * LSM6DSO32.h
 *
 *  Created on: Sep 6, 2024
 *      Author: liamt
 */

#ifndef INC_LSM6DSO32_H_
#define INC_LSM6DSO32_H_

#include "main.h"

typedef enum {
    LSM6DSO32_GYROSCOPE,
    LSM6DSO32_ACCELEROMETER
} LSM6DSO32_SENSOR;

typedef struct LSM6DSO32 {
    LSM6DSO32_SENSOR sensor;
    uint32_t timestamp;
    float X, Y, Z;
} LSM6DSO32;

typedef struct LSM6DSO32_OFFSET {
    float X, Y, Z;
} LSM6DSO32_OFFSET;

typedef enum {
    LSM6DSO32_SUCCESS,
    LSM6DSO32_FAILURE
} LSM6DSO32_STATUS;

typedef enum {
    LSM6DSO32_RAW,
    LSM6DSO32_COMPENSATED
} LSM6DSO32_DATA_TYPE;

LSM6DSO32_STATUS LSM6DSO32_Setup(SPI_HandleTypeDef * const hspi);
LSM6DSO32_STATUS LSM6DSO32_ReadGyro(LSM6DSO32 * const lsm6dso32_gyro,
                                    SPI_HandleTypeDef * const hspi,
                                    LSM6DSO32_DATA_TYPE const data_type);
LSM6DSO32_STATUS LSM6DSO32_ReadAccel(LSM6DSO32 * const lsm6dso32_accel,
                                     SPI_HandleTypeDef * const hspi,
                                     LSM6DSO32_DATA_TYPE const data_type);
LSM6DSO32_STATUS LSM6DSO32_UseOffset(LSM6DSO32 * const lsm6dso32,
                                     LSM6DSO32_OFFSET * const offset);
LSM6DSO32_STATUS LSM6DSO32_CalibrateOffset(LSM6DSO32 * const lsm6dso32,
                                           LSM6DSO32_OFFSET * const offset,
                                           SPI_HandleTypeDef * const hspi,
                                           LSM6DSO32_DATA_TYPE const data_type);

#endif /* INC_LSM6DSO32_H_ */
