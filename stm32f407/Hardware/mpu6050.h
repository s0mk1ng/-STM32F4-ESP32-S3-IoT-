#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "i2c.h"

/*===========================================================
 * MPU6050 I2C Address
 *
 * AD0 = 0:
 * 7-bit address = 0x68
 *
 * STM32 HAL uses the address shifted left by one bit:
 * 0x68 << 1 = 0xD0
 *==========================================================*/
#define MPU6050_ADDR_7BIT      0x68U
#define MPU6050_I2C_ADDR       (MPU6050_ADDR_7BIT << 1)

/*===========================================================
 * MPU6050 Register Map
 *==========================================================*/
#define MPU6050_REG_SMPLRT_DIV     0x19U  //采样率分频寄存器
#define MPU6050_REG_CONFIG         0x1AU  //配置寄存器
#define MPU6050_REG_GYRO_CONFIG    0x1BU  //陀螺仪配置寄存器
#define MPU6050_REG_ACCEL_CONFIG   0x1CU  //加速度计配置寄存器

//加速度计测量值寄存器
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU  //X轴的两组寄存器，由于存储为8个字节而每个轴的数据需要16位表示则有高8位与低8位
#define MPU6050_REG_ACCEL_XOUT_L   0x3CU
#define MPU6050_REG_ACCEL_YOUT_H   0x3DU  //Y轴的两组寄存器
#define MPU6050_REG_ACCEL_YOUT_L   0x3EU
#define MPU6050_REG_ACCEL_ZOUT_H   0x3FU  //Z轴的两组寄存器
#define MPU6050_REG_ACCEL_ZOUT_L   0x40U

#define MPU6050_REG_TEMP_OUT_H     0x41U  //温度测量值寄存器
#define MPU6050_REG_TEMP_OUT_L     0x42U

//陀螺仪测量值寄存器（同上加速度计测量值寄存器）
#define MPU6050_REG_GYRO_XOUT_H    0x43U  
#define MPU6050_REG_GYRO_XOUT_L    0x44U
#define MPU6050_REG_GYRO_YOUT_H    0x45U
#define MPU6050_REG_GYRO_YOUT_L    0x46U
#define MPU6050_REG_GYRO_ZOUT_H    0x47U
#define MPU6050_REG_GYRO_ZOUT_L    0x48U

#define MPU6050_REG_PWR_MGMT_1     0x6BU  //电源管理寄存器1
#define MPU6050_REG_PWR_MGMT_2     0x6CU  //电源管理寄存器2
#define MPU6050_REG_WHO_AM_I       0x75U  //验证设备寄存器

/* MPU6050 原始数据结构 */
typedef struct
{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
    
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

} MPU6050_RawData_t;

///* MPU6050 物理数据结构 */
//typedef struct
//{
//    float accel_x_g;
//    float accel_y_g;
//    float accel_z_g;

//    float gyro_x_dps;
//    float gyro_y_dps;
//    float gyro_z_dps;

//} MPU6050_PhysicalData_t;

typedef struct
{
	//陀螺仪解算的欧拉角
	float roll_g;
	float pitch_g;
  float yaw_g;
	//加速度计解算的欧拉角
	float roll_a; 
	float pitch_a;
		//互补滤波后的欧拉角
	float Roll;
	float Pitch;	
	float Yaw;

} MPU6050_PhysicalData_t;

HAL_StatusTypeDef MPU6050_Init(void);//初始化

HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i);//验证MPU

HAL_StatusTypeDef MPU6050_ReadRawData(MPU6050_RawData_t *data);//采集原始数据

HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data);//读mpu

float MPU6050_AccelRawToG(int16_t raw);//单个加速度值转换	

float MPU6050_GyroRawToDps(int16_t raw);//单个陀螺仪值转换

void MPU6050_ConvertRawData(const MPU6050_RawData_t *raw_data,MPU6050_PhysicalData_t *physical_data);//整组三轴数据转换


void MPU6050_PhysicalData(const MPU6050_RawData_t *raw_data,MPU6050_PhysicalData_t *physical_data);

#endif
