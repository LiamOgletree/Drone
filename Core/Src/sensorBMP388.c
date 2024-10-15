/*
 * BMP388.c
 *
 *  Created on: Jun 26, 2024
 *      Author: oglet
 */

#include "sensorBMP388.h"
#include "sensorShared.h"
#include "cmsis_os.h"

// TODO:
//  1. Add more precise error codes instead of blanket BMP388_FAILURE.
//  2. Refine HAL_TIMEOUT (set to ~100 ms right now).

/******************************/
/*           MACROS           */
/******************************/

#define WRITE               (0x00)
#define READ                (0x80)
#define GPIOx               (GPIOB)
#define GPIO_PINx           (GPIO_PIN_10)
#define REG_WHO_AM_I        (0x00)
#define REG_CALIBRATION     (0x31)
#define REG_PWR_CTRL        (0x1B)
#define REG_PRESSURE        (0x04)
#define REG_TEMPERATURE     (0x07)
#define ENABLE_PRESSURE     (0x01)
#define ENABLE_TEMPERATURE  (0x02)
#define NORMAL_MODE         (0x30)

/******************************/
/*          GLOBALS           */
/******************************/

// Latest temperature reading for pressure compensation function.
volatile float global_temperature;

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static void compute_compensations(BMP388_COMP * const bmp388_comp,
                                  uint8_t const * const DATA)
{
    // Compute hard-iron compensations according to BMP388 datasheet.
    bmp388_comp->T1 =  (float)((DATA[2] << 8) | DATA[1]) * 256.f;
    bmp388_comp->T2 =  (float)((DATA[4] << 8) | DATA[3]) / 1073741824.f;
    bmp388_comp->T3 =  (float)(DATA[5]) / 281474976710656.f;
    bmp388_comp->P1 = ((float)((DATA[7] << 8) | DATA[6]) - 16384.f) / 1048576.f;
    bmp388_comp->P2 = ((float)((DATA[9] << 8) | DATA[8]) - 16384.f) / 536870912.f;
    bmp388_comp->P3 =  (float)(DATA[10]) / 4294967296.f;
    bmp388_comp->P4 =  (float)(DATA[11]) / 137438953472.f;
    bmp388_comp->P5 =  (float)((DATA[13] << 8) | DATA[12]) * 8.f;
    bmp388_comp->P6 =  (float)((DATA[15] << 8) | DATA[14]) / 64.f;
    bmp388_comp->P7 =  (float)(DATA[16]) / 256.f;
    bmp388_comp->P8 =  (float)(DATA[17]) / 32768.f;
    bmp388_comp->P9 =  (float)((DATA[19] << 8) | DATA[18]) / 281474976710656.f;
    bmp388_comp->P10 = (float)(DATA[20]) / 281474976710656.f;
    bmp388_comp->P11 = (float)(DATA[21]) / 36893488147419103232.f;
}

static void compensate_temperature(BMP388 * const bmp388,
                                   BMP388_COMP const * const bmp388_comp,
                                   float const uncomp_temp)
{
    // Compute temperature using hard-iron compensations, as detailed
    //  in the BMP388 datasheet.
    float const partial_data1 = uncomp_temp - bmp388_comp->T1;
    bmp388->temperature = \
        (partial_data1 * bmp388_comp->T2) + \
        (partial_data1 * partial_data1) * bmp388_comp->T3;
}

static void compensate_pressure(BMP388 * const bmp388,
                                BMP388_COMP const * const bmp388_comp)
{
    // Compute pressure using hard-iron compensations and temperature,
    //  as detailed in the BMP388 datasheet.
    float const temperature = global_temperature;
    float const uncomp_pres = bmp388->pressure;
    float const partial_out1 = \
        (bmp388_comp->P5) + \
        (bmp388_comp->P6 * temperature) + \
        (bmp388_comp->P7 * temperature * temperature) + \
        (bmp388_comp->P8 * temperature * temperature * temperature);
    float const partial_out2 = uncomp_pres * \
        (bmp388_comp->P1 + \
        (bmp388_comp->P2 * temperature) + \
        (bmp388_comp->P3 * temperature * temperature) + \
        (bmp388_comp->P4 * temperature * temperature * temperature));
    float const partial_out3 = (uncomp_pres * uncomp_pres) * \
        (bmp388_comp->P9 + bmp388_comp->P10 * temperature) + \
        (uncomp_pres * uncomp_pres * uncomp_pres * bmp388_comp->P11);
    bmp388->pressure = partial_out1 + partial_out2 + partial_out3;
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

BMP388_STATUS BMP388_Setup(BMP388_COMP * const bmp388_comp,
                           SPI_HandleTypeDef * const hspi)
{
    uint8_t const TX1[] = { READ | REG_WHO_AM_I };
    uint8_t RX[22] = {0};

    // Conduct SPI transaction to retrieve constant WHO_AM_I
    //  BMP388 register value (0x50).
    if(sensor_tx_rx(TX1, sizeof(TX1) / sizeof(TX1[0]),
                    RX,  sizeof(RX)  / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return BMP388_FAILURE;
    }

    // If expected value is not received, return failure.
    if(RX[1] != 0x50) {
        return BMP388_WHOAMI_FAILURE;
    }

    // Otherwise, configure the BMP388 sensor and retrieve hard-iron
    //  offset values for compensating pressure and temperature.
    uint8_t const TX2[] = {
        WRITE | REG_PWR_CTRL,
        WRITE | ENABLE_PRESSURE | ENABLE_TEMPERATURE | NORMAL_MODE,
        READ  | REG_CALIBRATION
    };

    // Conduct SPI transaction.
    if(sensor_tx_rx(TX2, sizeof(TX2) / sizeof(TX2[0]),
                    RX,  sizeof(RX)  / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return BMP388_FAILURE;
    }

    // Translate raw hard-iron offsets to compensation values,
    //  according to the BMP388 datasheet.
    compute_compensations(bmp388_comp, RX);

    return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadTemp(BMP388 * const bmp388,
                              BMP388_COMP const * const bmp388_comp,
                              BMP388_DATA_TYPE const data_type,
                              SPI_HandleTypeDef * const hspi)
{
    uint8_t const TX[] = { READ | REG_TEMPERATURE };
    uint8_t RX[4] = {0};

    // Conduct SPI transaction to read BMP388 temperature data
    //  output registers.
    if(sensor_tx_rx(TX, sizeof(TX) / sizeof(TX[0]),
                    RX, sizeof(RX) / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return BMP388_FAILURE;
    }

    // Save time stamp to compute time steps for state machine.
    bmp388->timestamp = TIM5->CNT;

    // Compute uncompensated temperature.
    float const uncomp_temp = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

    // Compute compensated temperature and set the global temperature.
    compensate_temperature(bmp388, bmp388_comp, uncomp_temp);
    global_temperature = bmp388->temperature;

    // If reporting raw values instead of compensated values, set
    //  temperature to the uncompensated temperature.
    if(data_type == BMP388_RAW) {
        bmp388->temperature = uncomp_temp;
    }

    return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadPres(BMP388 * const bmp388,
                              BMP388_COMP const * const bmp388_comp,
                              BMP388_DATA_TYPE const data_type,
                              SPI_HandleTypeDef * const hspi)
{
    uint8_t const TX[] = { READ | REG_PRESSURE };
    uint8_t RX[4] = {0};

    // Conduct SPI transaction to read BMP388 pressure data
    //  output registers.
    if(sensor_tx_rx(TX, sizeof(TX) / sizeof(TX[0]),
                    RX, sizeof(RX) / sizeof(RX[0]),
                    hspi, GPIOx, GPIO_PINx)
            != SENSOR_SUCCESS) {
        return BMP388_FAILURE;
    }

    // Save time stamp to compute time steps for state machine.
    bmp388->timestamp = TIM5->CNT;

    // Compute uncompensated pressure.
    bmp388->pressure = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

    // If compensating pressure, run the pressure compensation function.
    if(data_type == BMP388_COMPENSATED) {
        compensate_pressure(bmp388, bmp388_comp);
    }

    return BMP388_SUCCESS;
}
