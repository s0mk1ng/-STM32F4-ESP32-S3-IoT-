#include "sht30.h"


/*===========================================================
 * Private definitions
 *==========================================================*/

/* SHT30 High Repeatability Measurement
 * Clock Stretching Disabled
 *
 * Command = 0x2400
 */
static const uint8_t SHT30_MEASURE_CMD[2] =
{
  SHT30_CMD_MEASURE_HIGH_REPEATABILITY_MSB,
  SHT30_CMD_MEASURE_HIGH_REPEATABILITY_LSB
};


/*===========================================================
 * SHT30_Init
 *==========================================================*/

HAL_StatusTypeDef SHT30_Init(void)
{
  /*
   * 检查 SHT30 是否能够正常响应 I2C 地址
   *
   * 当前默认地址：
   * 7-bit = 0x44
   * HAL   = 0x88
   */
  return HAL_I2C_IsDeviceReady(&hi2c1,SHT30_I2C_ADDR,3U,100U);
}

/*===========================================================
 * SHT30_CRC8
 *==========================================================*/

uint8_t SHT30_CRC8(const uint8_t *data,uint8_t length)
{
  uint8_t crc = 0xFFU;
	
  uint8_t i;
  uint8_t j;
	
  if (data == NULL)
  {
    return 0U;
  }
  for (i = 0U; i < length; i++)
    {
      crc ^= data[i];

      for (j = 0U; j < 8U; j++)
        {
          if (crc & 0x80U)
           {
						 crc = (uint8_t)((crc << 1U) ^ 0x31U);
           }
           else
           {
             crc <<= 1U;
           }
        }
    }
		
    return crc;
}

/*===========================================================
 * SHT30_StartMeasurement；发送测量命令后立即返回
 *==========================================================*/

HAL_StatusTypeDef SHT30_StartMeasurement(void)
{
  return HAL_I2C_Master_Transmit(&hi2c1,SHT30_I2C_ADDR,(uint8_t *)SHT30_MEASURE_CMD,2U,100U);
}

/*===========================================================
 * SHT30_ReadMeasurement；在外部等待测量完成后调用
 *==========================================================*/

HAL_StatusTypeDef SHT30_ReadMeasurement(SHT30_Data_t *data)
{
  HAL_StatusTypeDef status;

  uint8_t buffer[6];

  uint16_t temperature_raw;
  uint16_t humidity_raw;

  uint8_t temperature_crc;
  uint8_t humidity_crc; 
	
  if (data == NULL)
  {
		
    return HAL_ERROR;
  }


/* -----------------------------------------------------
* Read 6-byte measurement result
* ----------------------------------------------------- */
  status = HAL_I2C_Master_Receive(&hi2c1,SHT30_I2C_ADDR,buffer,6U,100U);
	

  if (status != HAL_OK)
  {
    return status;
  }

/* -----------------------------------------------------
* Temperature CRC
* ----------------------------------------------------- */

  temperature_crc =SHT30_CRC8(&buffer[0], 2U);

  if (temperature_crc != buffer[2])
  {
    return HAL_ERROR;
  }
	
/* -----------------------------------------------------
* Humidity CRC
* ----------------------------------------------------- */
	
  humidity_crc =SHT30_CRC8(&buffer[3], 2U);

  if (humidity_crc != buffer[5])
  {
    return HAL_ERROR;
  }


/* -----------------------------------------------------
* Raw data
* ----------------------------------------------------- */
  temperature_raw =((uint16_t)buffer[0] << 8U) |buffer[1];

  humidity_raw =((uint16_t)buffer[3] << 8U) |buffer[4];


/* -----------------------------------------------------
* Temperature
* T = -45 + 175 * ST / 65535
* ----------------------------------------------------- */
  data->temperature_c =-45.0f + 175.0f * ((float)temperature_raw / 65535.0f);


/* -----------------------------------------------------
* Humidity
* RH = 100 * SRH / 65535
* ----------------------------------------------------- */
  data->humidity_rh = 100.0f * ((float)humidity_raw / 65535.0f);

  return HAL_OK;
}
