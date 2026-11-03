/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "sensor_data.h"
#include "sht30.h"
#include "oled.h"
#include "math.h"
#include "stddef.h"
#include "usart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */
osMutexId_t telemetryMutexHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UI_EVENT_NEXT    ((uint8_t)1U)
#define UI_EVENT_PREV    ((uint8_t)2U)
#define VIBRATION_WINDOW_SAMPLES    100U
#define ACCEL_HPF_ALPHA             0.9845f

#define VIBRATION_WARN_ENTER_G       0.05f
#define VIBRATION_WARN_CLEAR_G       0.04f

#define VIBRATION_CRITICAL_ENTER_G   0.15f
#define VIBRATION_CRITICAL_CLEAR_G   0.12f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* MPU650*/
MPU6050_RawData_t g_mpu_data;

MPU6050_PhysicalData_t g_mpu_physical_data;

volatile HAL_StatusTypeDef g_mpu_status;

/* 队列消息 */
SensorMessage_t g_process_message;
//记录发送次数
volatile uint32_t g_sensor_queue_send_count = 0U;
//记录丢帧次数
volatile uint32_t g_sensor_queue_drop_count = 0U;

/* I2CMutex 错误统计变量 */
volatile uint32_t g_i2c1_mutex_error_count = 0U;

/* SHT30 */
//测试sht30用
SHT30_Data_t g_sht30_data;
volatile HAL_StatusTypeDef g_sht30_start_status;
volatile HAL_StatusTypeDef g_sht30_read_status;
volatile uint32_t g_sht30_read_count = 0U;
volatile uint32_t g_sht30_error_count = 0U;

/* OLED分块刷新调试变量 */
volatile uint32_t g_oled_mutex_retry_count = 0U;

volatile uint32_t g_oled_update_error_count = 0U;


/* ProcessTask */
// ProcessTask 接收到的传感器数据缓存
MPU6050_SensorData_t g_process_mpu_data;
SHT30_SensorData_t   g_process_sht30_data;
//g_telemetry作为ProcessTask 的共享数据
Telemetry_t g_telemetry;
//g_ui_telemetry是UITask自己拿到的快照
Telemetry_t g_ui_telemetry;
volatile uint32_t g_comm_tx_ok_count = 0U;
volatile uint32_t g_comm_tx_error_count = 0U;
volatile uint32_t g_comm_mutex_error_count = 0U;
volatile uint32_t g_comm_format_error_count = 0U;
//代表UI页数；规定0，1，2	
uint8_t g_ui_page = 0U;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ProcessTask */
osThreadId_t ProcessTaskHandle;
const osThreadAttr_t ProcessTask_attributes = {
  .name = "ProcessTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
uint8_t sensorQueueBuffer[ 10 * sizeof( SensorMessage_t ) ];
osStaticMessageQDef_t ssensorQueueControlBlock;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue",
  .cb_mem = &ssensorQueueControlBlock,
  .cb_size = sizeof(ssensorQueueControlBlock),
  .mq_mem = &sensorQueueBuffer,
  .mq_size = sizeof(sensorQueueBuffer)
};
/* Definitions for uiEventQueue */
osMessageQueueId_t uiEventQueueHandle;
const osMessageQueueAttr_t uiEventQueue_attributes = {
  .name = "uiEventQueue"
};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
osStaticMutexDef_t i2c1MutexControlBlock;
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex",
  .cb_mem = &i2c1MutexControlBlock,
  .cb_size = sizeof(i2c1MutexControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint8_t VibrationRms_Update(float accel_x_g,float accel_y_g,float accel_z_g,float *rms_out);

static VibrationState_t VibrationState_Update(VibrationState_t current_state,float vibration_rms_g);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartProcessTask(void *argument);
void StartUITask(void *argument);
void StartCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
	/* 初始化Telemetry基础信息 */
	g_telemetry.device_id = 0x0001;

	g_telemetry.frame_seq = 0;

	g_telemetry.timestamp_ms = 0;
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	telemetryMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (10, sizeof(SensorMessage_t), &sensorQueue_attributes);

  /* creation of uiEventQueue */
  uiEventQueueHandle = osMessageQueueNew (4, sizeof(uint8_t), &uiEventQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of ProcessTask */
  ProcessTaskHandle = osThreadNew(StartProcessTask, NULL, &ProcessTask_attributes);

  /* creation of UITask */
  UITaskHandle = osThreadNew(StartUITask, NULL, &UITask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	(void)argument;
	
	static const uint8_t uart_test_msg[] = "STM32 USART3 OK\r\n";

	(void)HAL_UART_Transmit(&huart3,(uint8_t *)uart_test_msg,(uint16_t)(sizeof(uart_test_msg) - 1U),100U);
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
	
  (void)argument;

  SensorMessage_t sensor_message;
	osStatus_t queue_status;
	
	uint8_t mpu6050_ready = 0U;

  uint32_t next_wake_time;
	
	uint32_t next_mpu_retry_time;

  uint32_t now;

  uint32_t next_sht30_start_time;

  uint32_t sht30_read_time;

  uint8_t sht30_measure_pending = 0U;

  const uint32_t sensor_period_ms = 5U;

  const uint32_t sht30_period_ms = 100U;

  const uint32_t sht30_measure_wait_ms = 20U;
	/* ==================================================
	* 1. MPU6050 初始化
	* ================================================== */
	
	if (osMutexAcquire(i2c1MutexHandle, 10U) == osOK)
		{
			if (MPU6050_Init() == HAL_OK)
				{
					g_mpu_status = HAL_OK;
					mpu6050_ready = 1U;
				}
				else
					{
						g_mpu_status = HAL_ERROR;
						mpu6050_ready = 0U;
					}
					
					(void)osMutexRelease(i2c1MutexHandle);
		}
		else
			{
				g_mpu_status = HAL_ERROR;
				mpu6050_ready = 0U;
				g_i2c1_mutex_error_count++;
			}

/* ==================================================
* 2. SHT30 初始化
* ================================================== */
  if (osMutexAcquire(i2c1MutexHandle, 10U) == osOK)
  {
    if (SHT30_Init() != HAL_OK)
      {
        g_sht30_error_count++;
      }
			
      osMutexRelease(i2c1MutexHandle);
  }
  else
  {
    g_sht30_error_count++;
  }


/* ==================================================
* 3. 初始化时间
* ================================================== */
  next_wake_time = osKernelGetTickCount();
	next_mpu_retry_time = next_wake_time + 1000U;

  next_sht30_start_time =next_wake_time + sht30_period_ms;

  sht30_read_time = 0U;
/* ==================================================
* 4. SensorTask 主循环
* ================================================== */
  for(;;)
  {
	  now = osKernelGetTickCount();
		
		if ((mpu6050_ready == 0U) && ((int32_t)(now - next_mpu_retry_time) >= 0))
			{
				if (osMutexAcquire(i2c1MutexHandle, 2U) == osOK)
					{
						if (MPU6050_Init() == HAL_OK)
							{
								mpu6050_ready = 1U;
								g_mpu_status = HAL_OK;
							}
							else
								{
									g_mpu_status = HAL_ERROR;
								}

								(void)osMutexRelease(i2c1MutexHandle);
					}
					else
						{
							g_i2c1_mutex_error_count++;
						}
						next_mpu_retry_time = now + 1000U;
			}


    /* ==================================================
    * A. MPU6050 200Hz
    * ================================================== */

    if (osMutexAcquire(i2c1MutexHandle, 2U) == osOK)
    {
      g_mpu_status =MPU6050_ReadRawData(&g_mpu_data);

      osMutexRelease(i2c1MutexHandle);

       if (g_mpu_status == HAL_OK)
        {
          /* -----------------------------
           * 姿态计算
           * ----------------------------- */         
					MPU6050_PhysicalData(&g_mpu_data,&g_mpu_physical_data);

           /* -----------------------------
            * 构造 MPU6050 消息
            * ----------------------------- */
          sensor_message.type =SENSOR_TYPE_MPU6050;

          sensor_message.timestamp =now;


          sensor_message.data.mpu6050.accel_x =g_mpu_data.accel_x;

          sensor_message.data.mpu6050.accel_y =g_mpu_data.accel_y;

          sensor_message.data.mpu6050.accel_z =g_mpu_data.accel_z;


          sensor_message.data.mpu6050.gyro_x =g_mpu_data.gyro_x;

          sensor_message.data.mpu6050.gyro_y =g_mpu_data.gyro_y;

          sensor_message.data.mpu6050.gyro_z =g_mpu_data.gyro_z;


          sensor_message.data.mpu6050.roll_g =g_mpu_physical_data.roll_g;

          sensor_message.data.mpu6050.pitch_g =g_mpu_physical_data.pitch_g;

          sensor_message.data.mpu6050.yaw_g =g_mpu_physical_data.yaw_g;


          sensor_message.data.mpu6050.roll_a =g_mpu_physical_data.roll_a;

          sensor_message.data.mpu6050.pitch_a =g_mpu_physical_data.pitch_a;


          sensor_message.data.mpu6050.Roll =g_mpu_physical_data.Roll;

          sensor_message.data.mpu6050.Pitch =g_mpu_physical_data.Pitch;

          sensor_message.data.mpu6050.Yaw =g_mpu_physical_data.Yaw;


           /* -----------------------------
            * 发送 MPU6050 数据
            * ----------------------------- */
           queue_status =osMessageQueuePut(sensorQueueHandle,&sensor_message,0U,0U);

           if (queue_status == osOK)
            {
							g_sensor_queue_send_count++;   
						}    
						else     
						{    
							g_sensor_queue_drop_count++;    
						}   
				}      
		}    
		else      
		{      
			g_i2c1_mutex_error_count++;      
		}


        
		/* ==================================================    
		* B. SHT30 100ms启动一次测量  
		* ================================================== */
   
		if ((sht30_measure_pending == 0U) &&((int32_t)(now - next_sht30_start_time) >= 0))  
		{    
			if (osMutexAcquire(i2c1MutexHandle, 2U) == osOK)
			{     
				g_sht30_start_status =SHT30_StartMeasurement();
      
				osMutexRelease(i2c1MutexHandle);
                
				if (g_sht30_start_status == HAL_OK)   
				{     
					sht30_measure_pending = 1U;

					sht30_read_time = now + sht30_measure_wait_ms;         
				}               
				else                							
				{                    
					g_sht30_error_count++;                
				}
                
				next_sht30_start_time +=                   
				sht30_period_ms;            
			}            
			else            
			{                
				g_i2c1_mutex_error_count++;            
			}       	
		}


        
		/* ==================================================        
		* C. SHT30 20ms以后读取结果     
		* ================================================== */
        	
		if ((sht30_measure_pending != 0U) &&((int32_t)(now - sht30_read_time) >= 0))        
		{            
			if (osMutexAcquire(i2c1MutexHandle, 2U) == osOK)           
			{               
				g_sht30_read_status =SHT30_ReadMeasurement(&g_sht30_data);
							
				osMutexRelease(i2c1MutexHandle);
														
				if (g_sht30_read_status == HAL_OK)								
				{				
					sensor_message.type = SENSOR_TYPE_SHT30;
    
					sensor_message.timestamp = now;
          
					sensor_message.data.sht30.temperature_c = g_sht30_data.temperature_c;
         
					sensor_message.data.sht30.humidity_rh = g_sht30_data.humidity_rh;
            
					queue_status = osMessageQueuePut(sensorQueueHandle,&sensor_message,0U,0U);
							
					if (queue_status == osOK)					
					{           					
						g_sensor_queue_send_count++;          			
					}          
					else            				
					{           
						g_sensor_queue_drop_count++;          
					}
      
					g_sht30_read_count++;    
				}  
				else
        
				{           
					g_sht30_error_count++;       
				}
    
				sht30_measure_pending = 0U;
			}   
			else   
			{     
				g_i2c1_mutex_error_count++;   
			}
}

        
/* ==================================================       
* D. 200Hz周期调度       
* ================================================== */
      
next_wake_time += sensor_period_ms;
      
(void)osDelayUntil(next_wake_time);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartProcessTask */
/**
* @brief Function implementing the ProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessTask */
void StartProcessTask(void *argument)
{
  /* USER CODE BEGIN StartProcessTask */
	(void)argument;
	
	osStatus_t status;
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;
	float vibration_rms_g;
	uint8_t vibration_rms_valid;

	float accel_magnitude_g;
	
	static VibrationState_t process_vibration_state = VIBRATION_STATE_UNKNOWN;
	
  /* Infinite loop */
  for(;;)
  {
		status = osMessageQueueGet(sensorQueueHandle,&g_process_message,NULL,osWaitForever);
      
		if (status == osOK)        
		{            
			switch (g_process_message.type)            
			{            
				case SENSOR_TYPE_MPU6050:
					
					/* 保存处理后的MPU6050数据 */
				
				g_process_mpu_data = g_process_message.data.mpu6050;
				/* Raw Acceleration -> g
				* 当前加速度量程为 ±2g
				* 灵敏度为 16384 LSB/g
				*/
				
				accel_x_g = (float)g_process_message.data.mpu6050.accel_x / 16384.0f;
				accel_y_g = (float)g_process_message.data.mpu6050.accel_y / 16384.0f;
				accel_z_g = (float)g_process_message.data.mpu6050.accel_z / 16384.0f;
				
				/* 三轴加速度模长 */
				
				accel_magnitude_g = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
				
				vibration_rms_valid = VibrationRms_Update(accel_x_g,accel_y_g,accel_z_g,&vibration_rms_g);
				
				if (vibration_rms_valid != 0U)
					{
						process_vibration_state = VibrationState_Update(process_vibration_state,vibration_rms_g);
					}
				
				/* 写入共享Telemetry之前先获取互斥锁 */
				
				if (osMutexAcquire(telemetryMutexHandle, 10U) == osOK)
					{
						g_telemetry.timestamp_ms = osKernelGetTickCount();
						
						g_telemetry.roll = g_process_message.data.mpu6050.Roll;
						
						g_telemetry.pitch = g_process_message.data.mpu6050.Pitch;
						
						g_telemetry.yaw = g_process_message.data.mpu6050.Yaw;
						
						g_telemetry.accel_x = g_process_message.data.mpu6050.accel_x;
						
						g_telemetry.accel_y = g_process_message.data.mpu6050.accel_y;
						
						g_telemetry.accel_z = g_process_message.data.mpu6050.accel_z;
						
						/* 保存换算后的物理量 */
						g_telemetry.accel_x_g = accel_x_g;
						g_telemetry.accel_y_g = accel_y_g;
						g_telemetry.accel_z_g = accel_z_g;
						
						/* 保存三轴合成加速度 */
						g_telemetry.accel_magnitude_g = accel_magnitude_g;
						
						
						g_telemetry.vibration_rms_g = vibration_rms_g;
						g_telemetry.vibration_state = process_vibration_state;
						
						if (vibration_rms_valid != 0U)
							{
								g_telemetry.valid_mask |= TELEMETRY_VALID_DERIVED;
							}
						
						g_telemetry.valid_mask |= TELEMETRY_VALID_MPU6050;
						/* 写完立即释放 */
						osMutexRelease(telemetryMutexHandle);
					}
					
					break;						
				
             
				case SENSOR_TYPE_SHT30:
					/* 将队列中的SHT30数据复制到处理缓存 */
				
				g_process_sht30_data = g_process_message.data.sht30;
				
				/* 写入共享Telemetry之前先获取互斥锁 */
				if (osMutexAcquire(telemetryMutexHandle, 10U) == osOK)
					{
						g_telemetry.timestamp_ms = osKernelGetTickCount();
						
						g_telemetry.temperature_c = g_process_message.data.sht30.temperature_c;
						
						g_telemetry.humidity_rh = g_process_message.data.sht30.humidity_rh;
						
						g_telemetry.valid_mask |= TELEMETRY_VALID_SHT30;
			
						/* 写完立即释放 */
						
						osMutexRelease(telemetryMutexHandle);
						
					}
					
					break;
              
              
				default:
					
				break;
         
			}     
		}
  }
  /* USER CODE END StartProcessTask */
}

/* USER CODE BEGIN Header_StartUITask */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/

static uint8_t UI_KeyIsPressed(uint8_t event)
{
    if (event == UI_EVENT_NEXT)
    {
        /* PA0：按下 = 1 */
        return (
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)
            == GPIO_PIN_SET
        );
    }

    if (event == UI_EVENT_PREV)
    {
        /* PB5：按下 = 0 */
        return (
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
            == GPIO_PIN_RESET
        );
    }

    return 0U;
}

static void UI_HandleKeyEvent(uint8_t event)
{
    /* 等待机械触点稳定 */
    osDelay(20U);

    /* 20ms后再次确认按键仍然处于按下状态 */
    if (UI_KeyIsPressed(event) == 0U)
    {
        return;
    }

    /* 一次有效按键只翻一页 */
    if (event == UI_EVENT_NEXT)
    {
        g_ui_page++;

        if (g_ui_page >= 3U)
        {
            g_ui_page = 0U;
        }
    }
    else if (event == UI_EVENT_PREV)
    {
        if (g_ui_page == 0U)
        {
            g_ui_page = 2U;
        }
        else
        {
            g_ui_page--;
        }
    }

    /* 等待按键完全松开 */
    while (UI_KeyIsPressed(event) != 0U)
    {
        osDelay(5U);
    }

    /* 清理可能残留的抖动事件 */
    (void)osMessageQueueReset(uiEventQueueHandle);
}

/* USER CODE END Header_StartUITask */
void StartUITask(void *argument)
{
  /* USER CODE BEGIN StartUITask */
	
	(void)argument;

	uint8_t event;
	osStatus_t ui_status;

	/* OLED分块刷新状态 */
	uint8_t oled_page = 0U;
	uint8_t oled_column = 0U;
	uint8_t oled_update_pending = 0U;

	uint32_t next_ui_refresh_tick = 0U;
	uint32_t now;

	HAL_StatusTypeDef oled_status;
	
  /* Infinite loop */
  for(;;)
  {		
/*--------------------------------------------------
* 0. 处理按键事件
*-------------------------------------------------*/
		
		ui_status = osMessageQueueGet(uiEventQueueHandle,&event,NULL,0U);
		
		if (ui_status == osOK)
		{
			UI_HandleKeyEvent(event);
		}
/*--------------------------------------------------
* 1. 获取Telemetry快照
*-------------------------------------------------*/
		
		now = osKernelGetTickCount();

		/* 只有上一帧刷新完成，才开始绘制下一帧 */
		if ((oled_update_pending == 0U) && ((int32_t)(now - next_ui_refresh_tick) >= 0))
			{
				next_ui_refresh_tick = now + 200U;
				
				if (osMutexAcquire(telemetryMutexHandle, 10U) == osOK)
					{
						g_ui_telemetry = g_telemetry;
						
						osMutexRelease(telemetryMutexHandle);
					}
			
/*--------------------------------------------------
* 2. 清空OLED显存
*-------------------------------------------------*/
			
			OLED_Clear();
			
			
			switch (g_ui_page)
				{
					/*======================================================
					* Page 0：System Overview
					*====================================================*/
				case 0U:
				OLED_ShowString(0,  0, "TEMP:");
				OLED_ShowString(0,  8, "HUM :");
				OLED_ShowString(0, 16, "ROLL:");
        OLED_ShowString(0, 24, "PITCH:");
        OLED_ShowString(0, 32, "YAW:");
        OLED_ShowString(0, 40, "MPU :");
        OLED_ShowString(0, 48, "SHT :");
        OLED_ShowString(0, 56, "PAGE 1/3");
				/*-------------------------
				* SHT30
				*-------------------------*/
				if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_SHT30) != 0U)
					{
						OLED_ShowFloat2(36,0,g_ui_telemetry.temperature_c);
						
						OLED_ShowString(72, 0, "C");
						
						OLED_ShowFloat2(36,8,g_ui_telemetry.humidity_rh);

            OLED_ShowString(72, 8, "%");
					}else
					{
						OLED_ShowString(36, 0, "----");
            OLED_ShowString(36, 8, "----");
					}
					/*-------------------------
					* MPU6050
					*-------------------------*/
					if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_MPU6050) != 0U)
						{
							OLED_ShowFloat2(30,16,g_ui_telemetry.roll);
							OLED_ShowFloat2(42,24,g_ui_telemetry.pitch);
							OLED_ShowFloat2(30,32,g_ui_telemetry.yaw);
							OLED_ShowString(36, 40, "OK");
						}
						else{
							OLED_ShowString(30, 16, "----");
							OLED_ShowString(42, 24, "----");
							OLED_ShowString(30, 32, "----");
							OLED_ShowString(36, 40, "--");
						}
						/*-------------------------
						* SHT30状态
						*-------------------------*/
						if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_SHT30) != 0U)
							{
								OLED_ShowString(36, 48, "OK");
							}else
							{

								OLED_ShowString(36, 48, "--");
							}
							break;
							/*======================================================
							* Page 1：MPU6050
							*====================================================*/
				case 1U:
				OLED_ShowString(0,  0, "MPU P2/3");
//				OLED_ShowString(54, 0, "P2/3");

        OLED_ShowString(0,  8, "ROLL:");
        OLED_ShowString(0, 16, "PITCH:");
        OLED_ShowString(0, 24, "YAW:");

        OLED_ShowString(0, 32, "ACCX:");
        OLED_ShowString(0, 40, "ACCY:");
        OLED_ShowString(0, 48, "ACCZ:");

//        OLED_ShowString(0, 56, "PAGE 2/3");
				
				if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_MPU6050) != 0U)
					{
						/* 姿态角 */
            OLED_ShowFloat2(36,8,g_ui_telemetry.roll);
						OLED_ShowFloat2(42,16,g_ui_telemetry.pitch);
						OLED_ShowFloat2(30,24,g_ui_telemetry.yaw);

            /* 加速度 Raw */
						OLED_ShowInt16(36,32,g_ui_telemetry.accel_x);
						
            OLED_ShowInt16(36,40,g_ui_telemetry.accel_y);

            OLED_ShowInt16(36,48,g_ui_telemetry.accel_z);

//            OLED_ShowString(42,56,"OK");
					}else
					{
						OLED_ShowString(36, 8, "----");
            OLED_ShowString(42, 16, "----");
            OLED_ShowString(30, 24, "----");

            OLED_ShowString(36, 32, "----");
            OLED_ShowString(36, 40, "----");
            OLED_ShowString(36, 48, "----");

//            OLED_ShowString(42,56,"--");
					}
					
					/*----------------------------------------------
					* 显示振动 RMS
					* 单位：g
					*---------------------------------------------*/
					OLED_ShowString(0, 56, "RMS:");
					
					if (((g_ui_telemetry.valid_mask & TELEMETRY_VALID_MPU6050) != 0U) && ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_DERIVED) != 0U))
						{
							OLED_ShowFloat2(30,56,g_ui_telemetry.vibration_rms_g);

							OLED_ShowString(66, 56, "G");
						}
						else
							{
								OLED_ShowString(30, 56, "----");
							}
							/* 显示振动状态 */

							if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_DERIVED) != 0U)
								{
									switch (g_ui_telemetry.vibration_state)
										{
										case VIBRATION_STATE_NORMAL:
											OLED_ShowString(82, 56, "NORM");
										break;

										case VIBRATION_STATE_WARNING:
											OLED_ShowString(82, 56, "WARN");
										break;

										case VIBRATION_STATE_CRITICAL:
											OLED_ShowString(82, 56, "CRIT");
										break;

										case VIBRATION_STATE_UNKNOWN:
										default:
											OLED_ShowString(82, 56, "INIT");
										break;
										}
								}
								else
									{
										OLED_ShowString(82, 56, "INIT");
									}
        break;
					
					/*======================================================
					* Page 2：SHT30
					*====================================================*/
    case 2U:

        OLED_ShowString(0,  0, "SHT30");
//        OLED_ShowString(36, 0, "P3/3");

        OLED_ShowString(0,  8, "TEMP:");
        OLED_ShowString(0, 16, "HUM :");
        OLED_ShowString(0, 24, "STATUS:");

        OLED_ShowString(0, 32, "DATA: LIVE");
        OLED_ShowString(0, 40, "ENV SENSOR");
        OLED_ShowString(0, 48, "SENSOR OK");
        OLED_ShowString(0, 56, "PAGE 3/3");

        if ((g_ui_telemetry.valid_mask & TELEMETRY_VALID_SHT30) != 0U)
					{
						OLED_ShowFloat2(36,8,g_ui_telemetry.temperature_c);

            OLED_ShowString(72,8,"C");

            OLED_ShowFloat2(36,16,g_ui_telemetry.humidity_rh);

            OLED_ShowString(72,16,"%");

            OLED_ShowString(42,24,"OK");
					}
					else{
						OLED_ShowString(36,8,"----");

            OLED_ShowString(36,16,"----");

            OLED_ShowString(42,24,"--");
					}

        break;
					
		default:

		g_ui_page = 0U;
		break;
				}
				/* 当前帧绘制完成，准备分块刷新 */
				oled_page = 0U;
				oled_column = 0U;
				oled_update_pending = 1U;
			} 
/* 结束200ms界面绘制周期 */
										
/*--------------------------------------------------
 * 5. OLED分块刷新
 * 每次只发送一个数据块
 *--------------------------------------------------*/

if (oled_update_pending != 0U)
{
    if (osMutexAcquire(i2c1MutexHandle, 2U) == osOK)
    {
        /* 发送当前数据块 */
        oled_status = OLED_UpdateChunk(
            oled_page,
            oled_column
        );

        /* 无论传输是否成功，都释放Mutex */
        (void)osMutexRelease(i2c1MutexHandle);

        if (oled_status == HAL_OK)
        {
            /* 当前数据块发送成功 */

            oled_column += OLED_UPDATE_CHUNK_SIZE;

            /* 当前Page发送完成 */
            if (oled_column >= OLED_WIDTH)
            {
                oled_column = 0U;
                oled_page++;

                /* 整屏刷新完成 */
                if (oled_page >= (OLED_HEIGHT / 8U))
                {
                    oled_update_pending = 0U;
                }
            }
        }
        else
        {
            /* 发送失败，保留当前位置，下次重试 */
            g_oled_update_error_count++;
        }
    }
    else
    {
        /* Mutex暂时不可用，下次循环重试 */
        g_oled_mutex_retry_count++;
    }
}

/* 让出CPU，给其他任务运行机会 */
osDelay(1U);

/*--------------------------------------------------
* 6. UI刷新周期 200ms = 5Hz
*-------------------------------------------------*/
				
    
  }
  /* USER CODE END StartUITask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
	(void)argument;
	
	Telemetry_t telemetry_snapshot;
	char tx_buf[160];
	int len;
	HAL_StatusTypeDef tx_status;
  /* Infinite loop */
  for(;;)
  {
		/* 1. 读取一致的遥测数据快照 */
		if (osMutexAcquire(telemetryMutexHandle, 10U) == osOK)
			{
				g_telemetry.frame_seq++;
				telemetry_snapshot = g_telemetry;

				osMutexRelease(telemetryMutexHandle);

				/* 2. 将遥测数据转换为文本帧 */
				len = snprintf(
				tx_buf,
				sizeof(tx_buf),
					"TLM,%u,%u,%lu,%ld,%ld,%ld,%d,%d,%d,%ld,%ld,%ld,%u,%u\r\n",
				(unsigned int)telemetry_snapshot.device_id,
					(unsigned int)telemetry_snapshot.frame_seq,
						(unsigned long)telemetry_snapshot.timestamp_ms,
							(long)(telemetry_snapshot.roll * 100.0f),
								(long)(telemetry_snapshot.pitch * 100.0f),
									(long)(telemetry_snapshot.yaw * 100.0f),
										(int)telemetry_snapshot.accel_x,
											(int)telemetry_snapshot.accel_y,
												(int)telemetry_snapshot.accel_z,
													(long)(telemetry_snapshot.temperature_c * 100.0f),
														(long)(telemetry_snapshot.humidity_rh * 100.0f),
															(long)(telemetry_snapshot.vibration_rms_g * 1000.0f),
																(unsigned int)telemetry_snapshot.vibration_state,
																	(unsigned int)telemetry_snapshot.valid_mask
																		);
																	/* 3. 发送完整文本帧 */
																	if ((len > 0) && ((size_t)len < sizeof(tx_buf)))
																		{
																			tx_status = HAL_UART_Transmit(
																			&huart3,
																			(uint8_t *)tx_buf,
																			(uint16_t)len,

																			100U
																			);

																			if (tx_status != HAL_OK)
																				{
																					g_comm_tx_error_count++;
																				}
																				else
																					{
																						g_comm_tx_ok_count++;
																					}

																		}
																		else
																			{
																				g_comm_format_error_count++;
																			}

			}
			else
				{
					g_comm_mutex_error_count++;
				}

        /* 约每100ms发送一次 */
        osDelay(100U);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*-----------------------------------------------------------
 * 三轴动态加速度 RMS
 *
 * 输入：三轴加速度，单位 g
 * 输出：100点窗口的三轴合成振动 RMS，单位 g
 *----------------------------------------------------------*/
static uint8_t VibrationRms_Update(float accel_x_g,float accel_y_g,float accel_z_g,float *rms_out)
	{
		static uint8_t initialized = 0U;
    static uint8_t result_valid = 0U;

    static float prev_input_x = 0.0f;
    static float prev_input_y = 0.0f;
    static float prev_input_z = 0.0f;

    static float prev_output_x = 0.0f;
    static float prev_output_y = 0.0f;
    static float prev_output_z = 0.0f;

    static float square_sum = 0.0f;
    static uint16_t sample_count = 0U;
    static float last_rms = 0.0f;

    float hpf_x;
    float hpf_y;
    float hpf_z;

    if (rms_out == NULL)
    {
        return 0U;
    }

    /* 初始化滤波器，避免上电时产生较大的启动尖峰 */
    if (initialized == 0U)
    {
        prev_input_x = accel_x_g;
        prev_input_y = accel_y_g;
        prev_input_z = accel_z_g;

        prev_output_x = 0.0f;
        prev_output_y = 0.0f;
        prev_output_z = 0.0f;

        initialized = 1U;
    }
    else
    {
        /* 三轴分别进行一阶高通滤波 */
        hpf_x = ACCEL_HPF_ALPHA *
                (prev_output_x + accel_x_g - prev_input_x);

        hpf_y = ACCEL_HPF_ALPHA *
                (prev_output_y + accel_y_g - prev_input_y);

        hpf_z = ACCEL_HPF_ALPHA *
                (prev_output_z + accel_z_g - prev_input_z);

        /* 保存本次输入与输出 */
        prev_input_x = accel_x_g;
        prev_input_y = accel_y_g;
        prev_input_z = accel_z_g;

        prev_output_x = hpf_x;
        prev_output_y = hpf_y;
        prev_output_z = hpf_z;

        /* 累积三轴动态加速度平方和 */
        square_sum +=
            hpf_x * hpf_x +
            hpf_y * hpf_y +
            hpf_z * hpf_z;

        sample_count++;

        /* 每100个样本更新一次RMS */
        if (sample_count >= VIBRATION_WINDOW_SAMPLES)
        {
            last_rms = sqrtf(
                square_sum /
                (float)VIBRATION_WINDOW_SAMPLES
            );

            square_sum = 0.0f;
            sample_count = 0U;

            result_valid = 1U;
        }
    }

    *rms_out = last_rms;

    return result_valid;
}
	
static VibrationState_t VibrationState_Update(VibrationState_t current_state,float vibration_rms_g)
{
    switch (current_state)
    {
        case VIBRATION_STATE_UNKNOWN:
        case VIBRATION_STATE_NORMAL:

            if (vibration_rms_g >=
                VIBRATION_CRITICAL_ENTER_G)
            {
                return VIBRATION_STATE_CRITICAL;
            }

            if (vibration_rms_g >=
                VIBRATION_WARN_ENTER_G)
            {
                return VIBRATION_STATE_WARNING;
            }

            return VIBRATION_STATE_NORMAL;


        case VIBRATION_STATE_WARNING:

            if (vibration_rms_g >=
                VIBRATION_CRITICAL_ENTER_G)
            {
                return VIBRATION_STATE_CRITICAL;
            }

            if (vibration_rms_g <
                VIBRATION_WARN_CLEAR_G)
            {
                return VIBRATION_STATE_NORMAL;
            }

            return VIBRATION_STATE_WARNING;


        case VIBRATION_STATE_CRITICAL:

            if (vibration_rms_g <
                VIBRATION_WARN_CLEAR_G)
            {
                return VIBRATION_STATE_NORMAL;
            }

            if (vibration_rms_g <
                VIBRATION_CRITICAL_CLEAR_G)
            {
                return VIBRATION_STATE_WARNING;
            }

            return VIBRATION_STATE_CRITICAL;


        default:

            return VIBRATION_STATE_UNKNOWN;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t event;
	
	if (GPIO_Pin == KEY_NEXT_Pin)
	{
		event = UI_EVENT_NEXT;
		
		(void)osMessageQueuePut(uiEventQueueHandle,&event,0U,0U);
	}
	else if(GPIO_Pin == KEY_PREV_Pin)
	{
		event = UI_EVENT_PREV;
		
		(void)osMessageQueuePut(uiEventQueueHandle,&event,0U,0U);
	}
}

/* USER CODE END Application */

