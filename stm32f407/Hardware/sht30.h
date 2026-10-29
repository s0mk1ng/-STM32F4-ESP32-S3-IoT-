#ifndef __SHT30_H
#define __SHT30_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>


/* =========================================================
 * I2C Address
 * ========================================================= */

#define SHT30_ADDR_7BIT    0x44U
#define SHT30_I2C_ADDR     (SHT30_ADDR_7BIT << 1)


/* =========================================================
 * Measurement Command
 * ========================================================= */

#define SHT30_CMD_MEASURE_HIGH_REPEATABILITY_MSB    0x24U
#define SHT30_CMD_MEASURE_HIGH_REPEATABILITY_LSB    0x00U


/* =========================================================
 * Measurement Data Length
 * ========================================================= */

#define SHT30_MEASURE_DATA_LENGTH    6U


/* =========================================================
 * SHT30 Data
 * ========================================================= */

typedef struct
{
    float temperature_c;
    float humidity_rh;

} SHT30_Data_t;


/* =========================================================
 * Function Prototypes
 * ========================================================= */

HAL_StatusTypeDef SHT30_Init(void);

HAL_StatusTypeDef SHT30_StartMeasurement(void);

HAL_StatusTypeDef SHT30_ReadMeasurement(SHT30_Data_t *data);

uint8_t SHT30_CRC8(const uint8_t *data,uint8_t length);


#endif
