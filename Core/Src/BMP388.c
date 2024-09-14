/*
 * BMP388.c
 *
 *  Created on: Jun 26, 2024
 *      Author: oglet
 */

#include "bmp388.h"
#include "cmsis_os.h"

static float temp_comp[3];
static float pres_comp[11];

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline void compute_compensations(uint8_t const * const DATA) {
    temp_comp[0] =  (float)((DATA[2] << 8) | DATA[1]) * 256.f;
    temp_comp[1] =  (float)((DATA[4] << 8) | DATA[3]) / 1073741824.f;
    temp_comp[2] =  (float)(DATA[5]) / 281474976710656.f;
    pres_comp[0] = ((float)((DATA[7] << 8) | DATA[6]) - 16384.f) / 1048576.f;
    pres_comp[1] = ((float)((DATA[9] << 8) | DATA[8]) - 16384.f) / 536870912.f;
    pres_comp[2] =  (float)(DATA[10]) / 4294967296.f;
    pres_comp[3] =  (float)(DATA[11]) / 137438953472.f;
    pres_comp[4] =  (float)((DATA[13] << 8) | DATA[12]) * 8.f;
    pres_comp[5] =  (float)((DATA[15] << 8) | DATA[14]) / 64.f;
    pres_comp[6] =  (float)(DATA[16]) / 256.f;
    pres_comp[7] =  (float)(DATA[17]) / 32768.f;
    pres_comp[8] =  (float)((DATA[19] << 8) | DATA[18]) / 281474976710656.f;
    pres_comp[9] = (float)(DATA[20]) / 281474976710656.f;
    pres_comp[10] = (float)(DATA[21]) / 36893488147419103232.f;
}

static inline void compensate_temperature(BMP388 * const bmp388,
                                          float const uncomp_temp) {
    float partial_data1 = uncomp_temp - temp_comp[0];
    bmp388->temperature = (partial_data1 * temp_comp[1]) +
                          (partial_data1 * partial_data1) * temp_comp[2];
}

static inline void compensate_pressure(BMP388 * const bmp388,
                                       float const uncomp_pres) {
    const float temp = bmp388->temperature;
    float partial_out1 = pres_comp[4] + (pres_comp[5] * temp) +
                         (pres_comp[6] * temp * temp) +
                         (pres_comp[7] * temp * temp * temp);
    float partial_out2 = uncomp_pres * (pres_comp[0] +
                         (pres_comp[1] * temp) +
                         (pres_comp[2] * temp * temp) +
                         (pres_comp[3] * temp * temp * temp));
    float partial_out3 = (uncomp_pres * uncomp_pres) *
                         (pres_comp[8] + pres_comp[9] * temp) +
                         (uncomp_pres * uncomp_pres * uncomp_pres *
                         pres_comp[10]);

    bmp388->pressure = partial_out1 + partial_out2 + partial_out3;
}

static HAL_StatusTypeDef transmit_receive(uint8_t const * const TX,
                                          uint8_t * const RX,
                                          SPI_HandleTypeDef * const hspi) {
    HAL_StatusTypeDef error;

    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_RESET);
    error = HAL_SPI_Transmit(hspi, (uint8_t *)TX, ARRAY_SIZE(TX), HAL_MAX_DELAY);
    if (error != HAL_OK)
        return error;

    error = HAL_SPI_Receive(hspi, (uint8_t *)RX, ARRAY_SIZE(RX), HAL_MAX_DELAY);
    if (error != HAL_OK)
        return error;

    HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

BMP388_STATUS BMP388_Setup(BMP388 * const bmp388,
                           SPI_HandleTypeDef * const hspi) {
    // If expected CHIP ID is not received, return failure
    const uint8_t TX1[1] = { READ | REG_WHO_AM_I };
    uint8_t RX[22] = {0};
    HAL_StatusTypeDef error;

    error = transmit_receive(TX1, RX, hspi);
    if (error != HAL_OK)
        return BMP388_FAILURE;

    if(RX[1] != 0x50) return BMP388_FAILURE;

    // Otherwise, configure sensor and receive calibration values
    const uint8_t TX2[3] = {
        WRITE | REG_PWR_CTRL,
        WRITE | PRES_ENABLE | TEMP_ENABLE | NORMAL_MODE,
        READ  | REG_CALIBRATION
    };

    error = transmit_receive(TX2, RX, hspi);
    if (error != HAL_OK)
        return BMP388_FAILURE;

    compute_compensations(RX);

    return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadTempPres(BMP388 * const bmp388,
                                  SPI_HandleTypeDef * const hspi) {
    const uint8_t TX[1] = { READ  | REG_OUTPUTS };
    uint8_t RX[7] = {0};
    float uncomp_temp, uncomp_pres;

    error = transmit_receive(TX, RX, hspi);
    if (error != HAL_OK)
        return BMP388_FAILURE;

    uncomp_temp = (float)((RX[6] << 16) | (RX[5] << 8) | RX[4]);
    uncomp_pres = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

    compensate_temperature(bmp388, uncomp_temp);
    compensate_pressure(bmp388, uncomp_pres);

    return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadTemp(BMP388 * const bmp388,
                              SPI_HandleTypeDef * const hspi) {
    const uint8_t TX[1] = { READ | REG_TEMP };
    uint8_t RX[4] = {0};
    float uncomp_temp;

    error = transmit_receive(TX, RX, hspi);
    if (error != HAL_OK)
        return BMP388_FAILURE;

    uncomp_temp = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

    compensate_temperature(bmp388, uncomp_temp);

    return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadPres(BMP388 * const bmp388,
                              SPI_HandleTypeDef * const hspi) {
    const uint8_t TX[1] = { READ | REG_PRES };
    uint8_t RX[4] = {0};
    float uncomp_pres;

    error = transmit_receive(TX, RX, hspi);
    if (error != HAL_OK)
        return BMP388_FAILURE;

    uncomp_pres = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

    compensate_pressure(bmp388, uncomp_pres);

    return BMP388_SUCCESS;
}
