#include "mpu6050.h"
#include <math.h>

volatile uint8_t g_mpu_raw_buffer[14];

/**
 * @brief  Check whether MPU6050 is present and wake it up.
 *
 * @return HAL_OK       Device detected and initialized.
 * @return HAL_ERROR    Device not detected or communication failed.
 */
HAL_StatusTypeDef MPU6050_Init(void)
{		
	/* 配置后的量程为加速度计±2 g；陀螺仪：  ±250 °/s */	
	/* 对应的灵敏度为加速度：16384 LSB/g；加速度：131 LSB/(°/s)；*/   
	HAL_StatusTypeDef status;	
	
	uint8_t data;

   
	/*-------------------------------------------------------   
  * Step 1:  
  * 检测MPU6050是否在线  
	*------------------------------------------------------*/  
	status = HAL_I2C_IsDeviceReady(&hi2c1,MPU6050_I2C_ADDR,3U,100U);
   
	/* If device does not respond, stop initialization. */	
	if (status != HAL_OK)
   
	{      
		return status;  
  }
   
	/*-------------------------------------------------------	
  * Step 2: 
  * 唤醒MPU6050	
	* PWR_MGMT_1 = 0x01
  * SLEEP = 0
  * CLKSEL = 001
	* 使用 X 轴陀螺仪 PLL 作为时钟源
	*------------------------------------------------------*/
	data = 0x01U;
	 
	status = HAL_I2C_Mem_Write(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_PWR_MGMT_1,I2C_MEMADD_SIZE_8BIT,&data,1U,100U);
		
	if (status != HAL_OK)	
	{
		return status;
	}
		
	//延迟10ms等待传感器稳定在freertos中最终使用osDelay(10);延时函数
	
	HAL_Delay(10U);
			 
	/*----------------------------------------------------------  
	* 3. 配置采样率
 * DLPF启用时基础输出速率为1000Hz
 * SMPLRT_DIV = 4
 * 采样率 = 1000 / (1 + 4) = 200Hz
 */
		
	data = 0x04U;
	
	status = HAL_I2C_Mem_Write(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_SMPLRT_DIV,I2C_MEMADD_SIZE_8BIT,&data,1U,100U);

	if (status != HAL_OK)
	{
		return status;
	}
	
		 
	/*----------------------------------------------------------
	* 4. 配置 DLPF
	*
	* CONFIG = 0x03
	* 启用数字低通滤波
	*----------------------------------------------------------*/
	
		
	data = 0x03U;
	
	status = HAL_I2C_Mem_Write(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1U,100U);
	
	if (status != HAL_OK)
	{
		return status;
	}
	
	/*---------------------------------------------------------
	* 5. 配置陀螺仪量程 
	*
	* FS_SEL = 0
	* ±250 °/s；±2000°/s
	*----------------------------------------------------------*/
	
	data  = 0x00U;
	
	status = HAL_I2C_Mem_Write(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_GYRO_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1U,100U);
	
	if (status != HAL_OK)
	{
		return status;
	}
	
	/*----------------------------------------------------------
	* 6. 配置加速度计量程
	*
  * AFS_SEL = 0

	* ±2 g；±16g
	*----------------------------------------------------------*/
	
	
	data = 0x00U;
	
	status = HAL_I2C_Mem_Write(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_ACCEL_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1U,100U);
	
	if (status != HAL_OK)
	{
		return status;
	}
	
  return HAL_OK;
}


/**
 * @brief  读取mpu6050的WHO_AM_I寄存器.
 *
 * @param  who_am_i 用于返回寄存器值的指针.
 *
 * @return HAL_OK     成功.
 * @return HAL_ERROR  返回无效参数或I2C通信故障.
 */
HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
	/* Check whether the output pointer is valid. */
	if (who_am_i == NULL)
  {
		return HAL_ERROR;
	}

   
	/*-------------------------------------------------------
  * Read register:
  *
  * WHO_AM_I = 0x75
	*
	* Expected value:
  *
	* 0x68
  *------------------------------------------------------*/
    
	return HAL_I2C_Mem_Read(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_WHO_AM_I,I2C_MEMADD_SIZE_8BIT,who_am_i,1U,100U);
	
}

HAL_StatusTypeDef MPU6050_ReadRawData(MPU6050_RawData_t *data)
{
	HAL_StatusTypeDef status;
	
	/* 防止传入空指针 */
	if (data == NULL)
	{
		return HAL_ERROR;
  }
	
	/*
  * 从 0x3B 开始连续读取 14 字节
  *
	* 0x3B ~ 0x40：加速度 XYZ，共 6 字节
	* 0x41 ~ 0x42：内部温度，共 2 字节（当前不使用）
	* 0x43 ~ 0x48：陀螺仪 XYZ，共 6 字节
	*/
   
	status = HAL_I2C_Mem_Read(&hi2c1,MPU6050_I2C_ADDR,MPU6050_REG_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT,(uint8_t *)g_mpu_raw_buffer,14U,100U);
	
	/* I2C 读取失败，直接返回错误 */
	if (status != HAL_OK) 
	{   
		return status;
  }
	
	/*  
	* 解析加速度数据 
	* 
	* 高字节在前，低字节在后
  */
	data->accel_x = (int16_t)((g_mpu_raw_buffer[0] << 8) | g_mpu_raw_buffer[1]);

	data->accel_y = (int16_t)((g_mpu_raw_buffer[2] << 8) | g_mpu_raw_buffer[3]);

	data->accel_z = (int16_t)((g_mpu_raw_buffer[4] << 8) | g_mpu_raw_buffer[5]);

    
	/* 
	* [6] [7] 是 MPU6050 内部温度原始数据
	*
	* 当前项目阶段暂时不使用，所以跳过。*/
	 
	/*解析陀螺仪数据*/
    
	data->gyro_x = (int16_t)((g_mpu_raw_buffer[8] << 8) | g_mpu_raw_buffer[9]);

	data->gyro_y = (int16_t)((g_mpu_raw_buffer[10] << 8) | g_mpu_raw_buffer[11]);

	data->gyro_z = (int16_t)((g_mpu_raw_buffer[12] << 8) | g_mpu_raw_buffer[13]);

	 return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    
	if (data == NULL)
	{    
		return HAL_ERROR;
	}

	return HAL_I2C_Mem_Read(&hi2c1,MPU6050_I2C_ADDR,reg,I2C_MEMADD_SIZE_8BIT,data,1U,100U);	
}


float roll_g,pitch_g,yaw_g;//陀螺仪解算的欧拉角
float roll_a,pitch_a;//加速度计解算的欧拉角
float Roll,Pitch,Yaw;//互补滤波后的欧拉角
	
void MPU6050_PhysicalData(const MPU6050_RawData_t *raw_data,MPU6050_PhysicalData_t *physical_data)
{
	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;

	const float dt = 0.005f;//采样周期
	const float alpha = 0.9f;//互补滤波器系数
	const float pi = 3.1415926f;
	
	if ((raw_data == NULL) || (physical_data == NULL))
	{
		return;
  }
	
	gyro_x_dps =(float)raw_data->gyro_x / 131.0f;
	
	gyro_y_dps =(float)raw_data->gyro_y / 131.0f;
	
	gyro_z_dps =(float)raw_data->gyro_z / 131.0f;
	
	//通过陀螺仪解算欧拉角
	physical_data->roll_g = physical_data->Roll + gyro_x_dps * dt;
	physical_data->pitch_g = physical_data->Pitch + gyro_y_dps * dt;
	physical_data->yaw_g = physical_data->Yaw + gyro_z_dps * dt;
	
	//通过加速度计解算欧拉角
	physical_data->pitch_a = atan2f((-1) * (float)raw_data->accel_x,(float)raw_data->accel_z) * 180.0f / pi;
	physical_data->roll_a = atan2f((float)raw_data->accel_y,(float)raw_data->accel_z) * 180.0f / pi;
	
	//通过互补滤波后得到的欧拉角
	physical_data->Roll =  alpha * physical_data->roll_g + (1.0f - alpha) * physical_data->roll_a;
	physical_data->Pitch =  alpha * physical_data->pitch_g + (1.0f - alpha) * physical_data->pitch_a;
	physical_data->Yaw =  alpha * physical_data->yaw_g;
}
