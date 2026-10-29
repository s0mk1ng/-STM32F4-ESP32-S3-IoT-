#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H
#define TELEMETRY_VALID_MPU6050    (1U << 0)
#define TELEMETRY_VALID_SHT30      (1U << 1)
#define TELEMETRY_VALID_INA219     (1U << 2)
#define TELEMETRY_VALID_DERIVED    (1U << 3)

#include <stdint.h>
#include "mpu6050.h"

typedef struct
{
    /* MPU6050 Raw Data */
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    /* MPU6050 Attitude Data */

    /* Gyroscope calculated Euler angles */
    float roll_g;
    float pitch_g;
    float yaw_g;

    /* Accelerometer calculated Euler angles */
    float roll_a;
    float pitch_a;

    /* Complementary filter result */
    float Roll;
    float Pitch;
    float Yaw;

} MPU6050_SensorData_t;

/* =========================
 * SHT30 数据
 * ========================= */
typedef struct
{
    float temperature_c;
    float humidity_rh;

} SHT30_SensorData_t;

/* =========================
 * INA219 数据
 * ========================= */
typedef struct
{
    float voltage_v;
    float current_a;
    float power_w;

} INA219_SensorData_t;


/* =========================
 * 传感器类型
 * ========================= */

typedef enum
{
    SENSOR_TYPE_MPU6050 = 0U,
    SENSOR_TYPE_SHT30,
    SENSOR_TYPE_INA219

} SensorType_t;

/* =========================
 * Sensor Queue 消息
 * ========================= */

typedef struct
{
    SensorType_t type;
    uint32_t timestamp;

    union
    {
      MPU6050_SensorData_t mpu6050;
        
			SHT30_SensorData_t sht30;
				
			INA219_SensorData_t  ina219;
       
    } data;

} SensorMessage_t;

typedef enum
{
    VIBRATION_STATE_UNKNOWN = 0U,
    VIBRATION_STATE_NORMAL,
    VIBRATION_STATE_WARNING,
    VIBRATION_STATE_CRITICAL
} VibrationState_t;

typedef struct
{
	 /* Communication Information */
	
	uint16_t device_id;

	uint16_t frame_seq;

	uint32_t timestamp_ms;


    /* MPU6050 */
    
	float roll;
	float pitch;  
	float yaw;
	
	
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	/* MPU6050 Derived Data */
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;
	
	float accel_magnitude_g;
	
	VibrationState_t vibration_state;
	
	float vibration_rms_g;
	

	/* SHT30 */
	float temperature_c;
	float humidity_rh;

	/* INA219 */
	float voltage_v;
	float current_a;
	float power_w;
	
	/* Valid mask */
	uint8_t valid_mask;

} Telemetry_t;

#endif
