/*
 * BMP388.h
 *
 *  Created on: Jun 26, 2024
 *      Author: oglet
 */

#ifndef SRC_BMP388_H_
#define SRC_BMP388_H_

#include "main.h"

/******************************/
/*           MACROS           */
/******************************/

#define WRITE           (0x00)
#define READ            (0x80)
#define GPIOx           (GPIOB)
#define GPIO_PINx       (GPIO_PIN_10)
#define REG_WHO_AM_I    (0x00)
#define REG_CALIBRATION (0x31)
#define REG_PWR_CTRL    (0x1B)
#define REG_OUTPUTS     (0x04)
#define REG_PRES        (0x04)
#define REG_TEMP        (0x07)
#define PRES_ENABLE     (0x01)
#define TEMP_ENABLE     (0x02)
#define NORMAL_MODE     (0x30)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef struct BMP388 {
    float temperature;
    float pressure;
} BMP388;

typedef enum {
    BMP388_SUCCESS,
    BMP388_FAILURE
} BMP388_STATUS;

BMP388_STATUS BMP388_Setup(BMP388 * const bmp388,
                           BMP388_COMP * const bmp388_comp,
                           SPI_HandleTypeDef * const hspi);
BMP388_STATUS BMP388_ReadTempPres(BMP388 * const bmp388,
                                  BMP388_COMP * const bmp388_comp,
                                  SPI_HandleTypeDef * const hspi);
BMP388_STATUS BMP388_ReadTemp(BMP388 * const bmp388,
                              BMP388_COMP * const bmp388_comp,
                              SPI_HandleTypeDef * const hspi);
BMP388_STATUS BMP388_ReadPres(BMP388 * const bmp388,
                              BMP388_COMP * const bmp388_comp,
                              SPI_HandleTypeDef * const hspi);

#endif /* SRC_BMP388_H_ */
