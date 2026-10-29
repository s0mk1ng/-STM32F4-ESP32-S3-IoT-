#include <stdio.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "mqtt_client.h"
#include <stdbool.h>
#include "esp_timer.h"
#include "wifi_credentials.h"

#define STM32_UART_PORT       UART_NUM_1
#define STM32_UART_TX_GPIO    GPIO_NUM_18
#define STM32_UART_RX_GPIO    GPIO_NUM_17
#define STM32_UART_BAUD_RATE  115200

#define UART_RX_BUF_SIZE      128


static char rx_line[192];
static size_t rx_line_len = 0;

static const char *TAG = "STM32_UART";
//记录当前链接状态
static volatile bool s_mqtt_connected = false;

/* MQTT 测试配置 */
#define MQTT_BROKER_URI "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC      "stm32demo/esp32s3_A83F/hello"
#define MQTT_TELEMETRY_TOPIC "stm32demo/esp32s3_A83F/telemetry"

static const char *MQTT_TAG = "MQTT";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static int s_hello_published = 0;

/* 函数声明 */
static void mqtt_start(void);

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
);


/**
 * @brief 初始化与 STM32 通信的 UART1
 */
static void stm32_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = STM32_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 安装 UART 驱动，只需要接收缓冲区 */
    ESP_ERROR_CHECK(
        uart_driver_install(
            STM32_UART_PORT,
            2048,
            0,
            0,
            NULL,
            0
        )
    );

    /* 配置串口通信参数 */
    ESP_ERROR_CHECK(
        uart_param_config(
            STM32_UART_PORT,
            &uart_config
        )
    );

    /* 设置 UART1 引脚：参数顺序是 TX、RX、RTS、CTS */
    ESP_ERROR_CHECK(
        uart_set_pin(
            STM32_UART_PORT,
            STM32_UART_TX_GPIO,
            STM32_UART_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    ESP_LOGI(TAG, "UART1 initialized");
    ESP_LOGI(TAG, "Baud rate: %d", STM32_UART_BAUD_RATE);
    ESP_LOGI(TAG, "TX: GPIO%d", STM32_UART_TX_GPIO);
    ESP_LOGI(TAG, "RX: GPIO%d", STM32_UART_RX_GPIO);
}

static void mqtt_start(void)
{
    /* 防止重复创建 MQTT 客户端 */
    if (s_mqtt_client != NULL)
    {
        return;
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (s_mqtt_client == NULL)
    {
        ESP_LOGE(MQTT_TAG, "MQTT client initialization failed");
        return;
    }

    ESP_ERROR_CHECK(
        esp_mqtt_client_register_event(
            s_mqtt_client,
            ESP_EVENT_ANY_ID,
            mqtt_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_mqtt_client_start(s_mqtt_client)
    );

    ESP_LOGI(MQTT_TAG, "MQTT client started");
}

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Wi-Fi started, connecting...");
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");

        esp_err_t err = esp_wifi_connect();

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Reconnect request failed: %s",
                     esp_err_to_name(err));
        }
    }
    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "Wi-Fi connected!");
        ESP_LOGI(TAG, "IP address: " IPSTR,IP2STR(&event->ip_info.ip));
        mqtt_start();
    }
}


static void wifi_init_sta(void)
{
    /* 初始化网络接口和事件循环 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 创建默认 Wi-Fi Station 网络接口 */
    esp_netif_create_default_wifi_sta();

    /* 初始化 Wi-Fi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册事件处理函数 */
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        )
    );

    /* 配置 Wi-Fi 名称和密码 */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config)
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete");
}


static void mqtt_event_handler(void *handler_args,esp_event_base_t base,int32_t event_id,void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch (event_id)
    {
        case MQTT_EVENT_CONNECTED:
        {
            s_mqtt_connected = true;

            ESP_LOGI(MQTT_TAG, "Connected to MQTT Broker");

            /* 只发布一次测试消息 */
            if (s_hello_published == 0)
            {
                int msg_id = esp_mqtt_client_publish(
                    s_mqtt_client,
                    MQTT_TOPIC,
                    "Hello MQTT from ESP32-S3",
                    0,
                    0,
                    0
                );

                if (msg_id >= 0)
                {
                    s_hello_published = 1;

                    ESP_LOGI(
                        MQTT_TAG,
                        "Hello MQTT publish accepted, msg_id=%d",
                        msg_id
                    );
                }
                else
                {
                    ESP_LOGE(
                        MQTT_TAG,
                        "Hello MQTT publish failed"
                    );
                }
            }

            break;
        }

        case MQTT_EVENT_DISCONNECTED:
        {
            s_mqtt_connected = false;

            ESP_LOGW(MQTT_TAG, "Disconnected from MQTT Broker");
            break;
        }

        case MQTT_EVENT_ERROR:
        {
            ESP_LOGE(MQTT_TAG, "MQTT connection error");
            break;
        }

        default:
            break;
    }
}



/**
 * @brief ESP-IDF 应用入口
 */
void app_main(void)
{
    static uint8_t rx_buffer[UART_RX_BUF_SIZE];
    char text_buffer[UART_RX_BUF_SIZE + 1] = {0};

    unsigned int device_id;
    unsigned int frame_seq;
    unsigned long timestamp_ms;
    
    long roll_cdeg;
    long pitch_cdeg;
    long yaw_cdeg;
    
    int accel_x;
    int accel_y;
    int accel_z;

    long temperature_centi;
    long humidity_centi;
    long vibration_milli_g;

    unsigned int vibration_state;
    unsigned int valid_mask;

    int parsed_fields;
    //保存即将发布的 JSON 数据
    char mqtt_payload[320];
    //记录格式化后的 JSON 长度
    int payload_len;
    //记录上一次发布的时间，避免每收到一帧就向服务器发送一条消息
    int64_t last_mqtt_publish_us = 0;

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    stm32_uart_init();

    wifi_init_sta();

    ESP_LOGI(TAG, "Waiting for STM32 data...");

    while (1)
    {
        int len = uart_read_bytes(
            STM32_UART_PORT,
            rx_buffer,
            UART_RX_BUF_SIZE,
            pdMS_TO_TICKS(1000)
        );


        
        for (int i = 0; i < len; i++)
        {
            char ch = (char)rx_buffer[i];

            if (ch == '\n')
            {
                /* 去掉末尾的 \r */
                if (rx_line_len > 0 &&
                    rx_line[rx_line_len - 1] == '\r')
                {
                    rx_line_len--;
                }

                rx_line[rx_line_len] = '\0';

                /* 只接受 TLM 开头的完整帧 */
               if (rx_line_len > 0 && strncmp(rx_line, "TLM,", 4) == 0)
               {
                parsed_fields = sscanf(
                    rx_line,
                    "TLM,%u,%u,%lu,%ld,%ld,%ld,%d,%d,%d,%ld,%ld,%ld,%u,%u",
                    &device_id,
                    &frame_seq,
                    &timestamp_ms,
                    &roll_cdeg,
                    &pitch_cdeg,
                    &yaw_cdeg,
                    &accel_x,
                    &accel_y,
                    &accel_z,
                    &temperature_centi,
                    &humidity_centi,
                    &vibration_milli_g,
                    &vibration_state,
                    &valid_mask
                );
                if (parsed_fields == 14)
                {
                     /* MPU6050 当前配置为 ±2g 时，灵敏度为 16384 LSB/g */
                    long accel_x_mg = ((long)accel_x * 1000L) / 16384L;
                    long accel_y_mg = ((long)accel_y * 1000L) / 16384L;
                    long accel_z_mg = ((long)accel_z * 1000L) / 16384L;
                    
                    ESP_LOGI(
                        TAG,
                        "Parsed: dev=%u seq=%u ts=%lu ms "
                        "roll=%ld cdeg pitch=%ld cdeg yaw=%ld cdeg "
                        "accel=(%ld,%ld,%ld) mg "
                        "temp=%ld.%02ld C humidity=%ld.%02ld %% "
                        "rms=%ld mg state=%u mask=0x%02X",
                        device_id,
                        frame_seq,
                        timestamp_ms,
                        roll_cdeg,
                        pitch_cdeg,
                        yaw_cdeg,
                        accel_x_mg,
                        accel_y_mg,
                        accel_z_mg,
                        temperature_centi / 100L,
                        labs(temperature_centi % 100L),
                        humidity_centi / 100L,
                        labs(humidity_centi % 100L),
                        vibration_milli_g,
                        vibration_state,
                        valid_mask
                    );

                    
                    /* 每秒最多发布一条遥测数据 */
                    int64_t now_us = esp_timer_get_time();

                    if (s_mqtt_connected &&
                        (now_us - last_mqtt_publish_us >= 200000LL))
                    {
                        last_mqtt_publish_us = now_us;

                        /* 将解析后的数据封装成 JSON */
                        payload_len = snprintf(
                            mqtt_payload,
                            sizeof(mqtt_payload),

                            "{\"device_id\":%u,"
                            "\"frame_seq\":%u,"
                            "\"timestamp_ms\":%lu,"
                            "\"roll_deg\":%s%ld.%02ld,"
                            "\"pitch_deg\":%s%ld.%02ld,"
                            "\"yaw_deg\":%s%ld.%02ld,"
                            "\"accel_x_mg\":%ld,"
                            "\"accel_y_mg\":%ld,"
                            "\"accel_z_mg\":%ld,"
                            "\"temperature_c\":%s%ld.%02ld,"
                            "\"humidity_rh_pct\":%s%ld.%02ld,"
                            "\"vibration_rms_g\":%s%ld.%03ld,"
                            "\"vibration_state\":%u,"
                            "\"valid_mask\":%u}",

                            device_id,
                            frame_seq,
                            timestamp_ms,

                            roll_cdeg < 0 ? "-" : "",
                            labs(roll_cdeg) / 100L,
                            labs(roll_cdeg) % 100L,

                            pitch_cdeg < 0 ? "-" : "",
                            labs(pitch_cdeg) / 100L,
                            labs(pitch_cdeg) % 100L,

                            yaw_cdeg < 0 ? "-" : "",
                            labs(yaw_cdeg) / 100L,
                            labs(yaw_cdeg) % 100L,

                            accel_x_mg,
                            accel_y_mg,
                            accel_z_mg,

                            temperature_centi < 0 ? "-" : "",
                            labs(temperature_centi) / 100L,
                            labs(temperature_centi) % 100L,

                            humidity_centi < 0 ? "-" : "",
                            labs(humidity_centi) / 100L,
                            labs(humidity_centi) % 100L,

                            vibration_milli_g < 0 ? "-" : "",
                            labs(vibration_milli_g) / 1000L,
                            labs(vibration_milli_g) % 1000L,

                            vibration_state,
                            valid_mask
                        );

                        /* 确认 JSON 完整放入缓冲区 */
                        if (payload_len > 0 &&
                            (size_t)payload_len < sizeof(mqtt_payload))
                        {
                            int msg_id = esp_mqtt_client_publish(
                                s_mqtt_client,
                                MQTT_TELEMETRY_TOPIC,
                                mqtt_payload,
                                0,
                                0,
                                0
                            );

                            if (msg_id >= 0)
                            {
                                ESP_LOGI(
                                    MQTT_TAG,
                                    "Telemetry published: seq=%u, msg_id=%d",
                                    frame_seq,
                                    msg_id
                                );
                            }
                            else
                            {
                                ESP_LOGW(
                                    MQTT_TAG,
                                    "Telemetry publish failed, seq=%u",
                                    frame_seq
                                );
                            }
                        }
                        else
                        {
                            ESP_LOGE(MQTT_TAG, "JSON formatting failed or buffer too small");
                        }
                    }


                }
                else
                {
                    ESP_LOGW(
                        TAG,
                        "字段数量错误: parsed=%d, frame=%s",
                        parsed_fields,
                        rx_line
                    );
                }
            }
            else if (rx_line_len > 0)
            {
                ESP_LOGW(TAG, "无效数据帧");
            }
            
            /* 开始接收下一帧 */
            rx_line_len = 0;
        }
        else if (rx_line_len < sizeof(rx_line) - 1)
        {
            rx_line[rx_line_len++] = ch;
        }
        else
        {
            /* 超长帧，丢弃并重新同步 */
            rx_line_len = 0;
            ESP_LOGW(TAG, "数据帧过长，重新同步");
        }
    }


        // if (len > 0)
        // {
        //     ESP_LOGI(TAG, "Received %d bytes", len);

        //     /* 输出原始字节，方便后续调试二进制协议 */
        //     ESP_LOG_BUFFER_HEXDUMP(
        //         TAG,
        //         rx_buffer,
        //         len,
        //         ESP_LOG_INFO
        //     );

        //     /* 将可打印字符保留下来，其他字节显示为点 */
        //     for (int i = 0; i < len; i++)
        //     {
        //         if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126)
        //         {
        //             text_buffer[i] = (char)rx_buffer[i];
        //         }
        //         else
        //         {
        //             text_buffer[i] = '.';
        //         }
        //     }

        //     text_buffer[len] = '\0';

        //     ESP_LOGI(TAG, "ASCII: %s", text_buffer);
        // }
        
    }
}
