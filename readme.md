# 工业 IoT 网关项目技术设计与开发同步文档

**V1.3｜基于 2026-11-03 当前源码的开发基线**

STM32F407VET6 + FreeRTOS + MPU6050 + SHT30 + OLED + ESP32-S3 + UART + Wi-Fi + MQTT

> 用途：当前实现归档、后续开发连续性、项目验收与答辩复盘。  
> 固定约束：**INA219 永久排除**，不进入硬件、驱动、队列、Telemetry、UART 协议、MQTT JSON 或未来升级计划。

## 0. 项目基线与开发约定

- CubeMX 负责外设、GPIO、时钟、FreeRTOS Task/Queue/Mutex 等配置对象；业务逻辑优先放在 USER CODE 区域。
- 引脚与电气连接以实际 STM32F407 核心板原理图和已验证连线为准。
- 每次只引入一个主要变化：修改一个模块后先编译、烧录、实机验证，再进入下一模块。
- STM32 负责现场实时采集、边缘处理和本地显示；ESP32-S3 负责网络接入与 MQTT 数据转发。
- 当前 UART 是 ASCII 文本 TLM 帧；AA55 + CRC16 二进制协议属于后续升级，不是当前实现。

## 1. 项目目标与系统边界

项目目标是构建双 MCU 工业现场监测网关原型。STM32F407VET6 负责传感器采集、姿态与振动边缘处理、OLED 本地 HMI 和遥测帧发送；ESP32-S3 负责 UART 接收解析、Wi-Fi STA 联网和 MQTT JSON 发布。后续再接入 ThingsBoard、离线存储、设备控制和系统健康监测。

| 层级 | 当前实现 | 后续升级 |
|---|---|---|
| 传感器/驱动 | MPU6050、SHT30；I2C1 共享总线 | 校准、异常恢复、采样质量监测 |
| 实时控制 MCU | STM32F407VET6 + FreeRTOS | StorageTask、ControlTask、WatchdogTask |
| 本地 HMI | SSD1306 128×64 OLED，三页面和按键翻页 | 告警页面、设备健康状态、参数配置 |
| MCU 间通信 | USART3 → ESP32 UART1，ASCII TLM 文本帧 | 带版本、长度、序号、CRC16 的二进制协议；必要时 ACK/重传 |
| 网络层 | ESP32-S3 Wi-Fi STA + MQTT 客户端 | TLS、认证、QoS1、断线缓存/补发 |
| 云端 | 公共 MQTT Broker 和 Topic | ThingsBoard Telemetry/Attributes/RPC/Dashboard/Alarm |
| 本地持久化 | 尚未接入 | W25Q64 + StorageTask + Store-and-Forward |

## 2. 系统总体架构与数据流

### 2.1 现场采集与本地显示

```text
MPU6050 ─┐
         ├─ I2C1 ─ SensorTask ─ SensorMessage_t ─ sensorQueue
SHT30 ───┘                                      │
                                               ▼
                                           ProcessTask
                                               │
                         姿态数据 / 加速度换算 / 振动 RMS / 状态判断
                                               │
                                               ▼
                                           Telemetry_t
                                               │ telemetryMutex
                                               ▼
                                            UITask
                                               │ i2c1Mutex
                                               ▼
                                        SSD1306 OLED
```

### 2.2 远程遥测链

```text
Telemetry_t
    │
    ▼
CommTask（约 100 ms 一次）
    │ USART3 / huart3 / 115200 baud
    ▼
STM32F407 ── ASCII TLM,...\r\n ──> ESP32-S3 UART1
                                      │
                                      ▼
                                14 字段 sscanf 解析
                                      │
                                      ▼
                                 JSON 组包
                                      │
                               Wi-Fi STA / MQTT
                                      │
                                      ▼
                                  MQTT Broker
```

两条链路相互解耦：OLED 从 Telemetry 快照显示数据；CommTask 从共享 Telemetry 读取快照并发送。ESP32 的联网和 MQTT 发布不得阻塞 STM32 的传感器采集。

## 3. 硬件组成与资源基线

| 器件/资源 | 用途 | 状态 |
|---|---|---|
| STM32F407VET6 | 现场实时主控 | 当前主控 |
| MPU6050 | 三轴加速度、三轴角速度、姿态估算、振动输入 | 当前采集链 |
| SHT30 | 温度、相对湿度 | 当前采集链 |
| SSD1306 OLED 128×64 | 本地三页面 HMI | 当前显示链 |
| ESP32-S3 | UART 接收、Wi-Fi、MQTT | 当前网络侧源码已实现 |
| W25Q64 | 后续短期缓存和 Store-and-Forward | 规划接入，尚未并入当前主链 |
| INA219 | — | **永久排除** |

### 3.1 引脚记录（沿用 V1.2 项目记录，改板/接线前须复核）

| 引脚/资源 | 用途 | 说明 |
|---|---|---|
| PB6 / PB7 | I2C1 SCL / SDA | MPU6050、SHT30、OLED 共享总线 |
| PB10 / PB11 | USART3 TX / RX | STM32 与 ESP32-S3 UART 连接 |
| PA0 / PB5 | 下一页 / 上一页按键 | PA0 高电平按下；PB5 低电平按下 |
| PB12–PB15 | SPI2 CS/SCK/MISO/MOSI | W25Q64 规划资源，当前未启用 |
| PE0 / PE1 | 继电器 / 蜂鸣器 | 后续 ControlTask 规划 |
| PA8 | TIM1_CH1 PWM | 后续执行器/PWM 规划 |
| PA13 / PA14 | SWDIO / SWCLK | 下载与调试，保留 |

历史时钟基线（来自 V1.2 记录）：HSE 8 MHz，SYSCLK 168 MHz，AHB 168 MHz，APB1 42 MHz，APB2 84 MHz，HAL Timebase TIM6，CMSIS-RTOS2 / CMSIS_V2，FreeRTOS Tick 1000 Hz。应以当前 `.ioc` 和实机配置再次核对。

## 4. STM32 FreeRTOS 软件架构

| 任务 | 优先级 | 栈配置 | 主要职责 |
|---|---|---:|---|
| SensorTask | AboveNormal | 2048 bytes | MPU6050 约 200 Hz；SHT30 约 10 Hz 两阶段测量；投递 sensorQueue |
| ProcessTask | Normal | 2048 bytes | 阻塞等待 sensorQueue；换算物理量；振动 RMS/状态判断；更新 Telemetry_t |
| UITask | BelowNormal | 2048 bytes | Telemetry 快照；uiEventQueue；OLED 三页面和分块刷新 |
| CommTask | BelowNormal | 3072 bytes | 读取 Telemetry 快照；格式化 TLM 文本帧；USART3 发送 |
| defaultTask | Low | 512 bytes | 后台占位；建议删除一次性 UART 测试输出 |

队列与互斥锁：

- `sensorQueue`：长度 10，消息类型 `SensorMessage_t`；SensorTask → ProcessTask。
- `uiEventQueue`：长度 4，消息类型 `uint8_t`；按键 EXTI → UITask。
- `i2c1Mutex`：保护 MPU6050、SHT30、OLED 对共享 I2C1 的访问。
- `telemetryMutex`：保护 ProcessTask 写入、UITask/CommTask 读取 Telemetry_t 的一致性。

当前任务优先级关系：SensorTask > ProcessTask > UITask = CommTask > defaultTask。

## 5. 传感器处理与边缘算法

### 5.1 MPU6050

- SensorTask 基础周期 5 ms，约 200 Hz。
- 当前数据结构保存陀螺仪积分角、加速度姿态角以及互补滤波后的 Roll/Pitch/Yaw。
- 当前加速度量程按 ±2 g 处理，换算灵敏度为 16384 LSB/g。
- ProcessTask 将 raw accel 换算为 g，并计算三轴加速度模长。
- Yaw 仅由陀螺仪积分时会随时间漂移；没有磁力计时不能解释为绝对磁北航向。

驱动层量程和陀螺仪 Raw→°/s 比例系数需要在最新 `mpu6050.c` 中复核。V1.2 历史审查曾记录 ±250 °/s 与 `/16.4f` 换算不一致、应核对 131 LSB/(°/s)；本次未上传 `mpu6050.c`，不能据此断言当前驱动已修正。

### 5.2 SHT30

- SensorTask 每约 100 ms 启动一次测量，等待约 20 ms 后读取温湿度，目标约 10 Hz。
- 与 MPU6050、OLED 共用 I2C1，所有访问必须遵守 `i2c1Mutex`。

### 5.3 振动 RMS

- 高通滤波系数：`ACCEL_HPF_ALPHA = 0.9845`。
- 窗口：100 点；SensorTask 约 200 Hz，因此约 0.5 s 更新一次 RMS。
- `RMS = sqrt(sum(ax_hpf² + ay_hpf² + az_hpf²) / 100)`。

| 状态转换 | 阈值 |
|---|---:|
| 进入 WARNING | RMS ≥ 0.05 g |
| WARNING 恢复 NORMAL | RMS < 0.04 g |
| 进入 CRITICAL | RMS ≥ 0.15 g |
| CRITICAL 降级 WARNING | RMS < 0.12 g；若低于 0.04 g 则直接恢复 NORMAL |

这些是当前代码参数，不代表已经完成行业标准标定。后续应使用实际设备工况数据确定阈值。

## 6. Telemetry 数据结构与有效位

清理 INA219 后，`Telemetry_t` 保留通信元信息、MPU6050 姿态与加速度、派生加速度指标、振动 RMS/状态、SHT30 温湿度和 `valid_mask`。

```c
#define TELEMETRY_VALID_MPU6050    (1U << 0)
#define TELEMETRY_VALID_SHT30      (1U << 1)
/* bit2 保留 */
#define TELEMETRY_VALID_DERIVED    (1U << 3)
```

保留 DERIVED 为 bit3 是为了不改变当前有效位编码。删除 INA219 后 bit2 作为保留位。

## 7. STM32 ↔ ESP32-S3 当前 UART 协议

当前实现为 ASCII 文本行协议，不是二进制帧。STM32 `CommTask` 从 `Telemetry_t` 复制快照，使用 `snprintf()` 生成以 `TLM,` 开头、以 CRLF 结束的文本，再通过 `huart3` 调用 `HAL_UART_Transmit()` 发送。

```text
TLM,device_id,frame_seq,timestamp_ms,
roll_cdeg,pitch_cdeg,yaw_cdeg,
accel_x,accel_y,accel_z,
temperature_centi,humidity_centi,
vibration_milli_g,vibration_state,valid_mask\r\n
```

| 字段 | 当前编码/单位 |
|---|---|
| device_id | 设备编号 |
| frame_seq | 帧序号 |
| timestamp_ms | 毫秒 |
| roll/pitch/yaw | 角度 × 100 |
| accel_x/y/z | MPU6050 原始整数 |
| temperature | °C × 100 |
| humidity | %RH × 100 |
| vibration_rms | g × 1000，即 milli-g |
| vibration_state | 状态枚举值 |
| valid_mask | 有效位掩码 |

STM32 CommTask 约每 100 ms 发送一次，约 10 Hz。ESP32 使用 UART_NUM_1、GPIO18 TX、GPIO17 RX、115200 baud、8N1 接收，按换行符组帧，再用 `sscanf()` 解析 14 个字段。

当前协议没有 CRC、ACK、重传、版本协商或数值范围校验；STM32 采用阻塞式 `HAL_UART_Transmit()`，尚未升级为 DMA/中断发送。

## 8. ESP32-S3 网络与 MQTT 当前实现

- 初始化 NVS；必要时处理无可用页/版本变化。
- Wi-Fi Station 模式接入网络；断开时请求重连，获得 IP 后启动 MQTT 客户端。
- 当前 Broker：`mqtt://broker.emqx.io:1883`，公共测试 Broker，明文 TCP，不具备生产环境所需的 TLS/认证配置。
- 当前遥测 Topic：`stm32demo/esp32s3_A83F/telemetry`。
- ESP32 将 TLM 转为 JSON：`device_id`、`frame_seq`、`timestamp_ms`、`roll_deg`、`pitch_deg`、`yaw_deg`、`accel_x_mg`、`accel_y_mg`、`accel_z_mg`、`temperature_c`、`humidity_rh_pct`、`vibration_rms_g`、`vibration_state`、`valid_mask`。
- 当前 MQTT 发布间隔为 200000 μs，即 200 ms，最大约 5 Hz；源码注释“每秒最多一条”与实际常量不一致，应修正。
- 当前 publish 参数使用 QoS 0、retain 0。
- Wi-Fi 凭据由 `wifi_credentials.h` 提供；真实 SSID/密码不得提交到公开仓库。

源码具备 UART→解析→JSON→MQTT 的实现路径，但代码审查不能证明 Broker 已持续收到数据。应使用 MQTTX 等订阅客户端完成端到端验收并保存日志/截图。

## 9. 当前 DEMO 阶段验收状态

| 功能项 | 当前判断 |
|---|---|
| MPU6050 采集与姿态计算 | 已实现；此前开发记录有实机运行结果 |
| SHT30 温湿度 | 已实现；此前开发记录有实机运行结果 |
| FreeRTOS 队列/互斥锁 | 已实现；此前已验证 |
| OLED 三页面和按键翻页 | 已实现；用户此前反馈修改后屏幕显示正常 |
| I2C 互斥锁稳定性 | 用户此前反馈 `g_i2c1_mutex_error_count=0`、`g_oled_mutex_retry_count=0`；清理后需回归 |
| STM32 USART3 TLM 发送 | 源码已实现；需记录连续运行结果 |
| ESP32 UART 解析 | 源码已实现；需记录真实接收日志 |
| Wi-Fi/MQTT JSON 发布 | 源码已实现；需用订阅端确认持续收数 |
| ThingsBoard / W25Q64 | 未实现 |

**阶段结论：** 项目已进入具备完整功能链代码的 DEMO/MVP 阶段。若 MQTTX 能持续收到字段正确、序号递增的 JSON，且断网重连和长时间运行通过，即可正式验收当前端到端 DEMO；当前还不能称为工业级可靠性产品。

## 10. 本轮代码清理清单

### 10.1 `sensor_data.h`

删除：

- `TELEMETRY_VALID_INA219`
- `INA219_SensorData_t`
- `SENSOR_TYPE_INA219`
- `SensorMessage_t.data.ina219`
- `Telemetry_t.voltage_v/current_a/power_w`

保留 `TELEMETRY_VALID_DERIVED` 为 bit3；bit2 保留。

### 10.2 `freertos.c`

建议删除无引用或仅用于调试的数据：

- `g_sensor_queue_count`
- `g_sht30_data_valid`
- `g_vibration_sq_sum`
- `g_vibration_sample_count`
- `g_process_mpu_data` / `g_process_sht30_data` 及对应赋值
- `g_process_mpu_count` / `g_process_sht30_count` 及递增语句
- 第二处重复的 `VIBRATION_WINDOW_SAMPLES`

建议保留故障/队列/通信诊断计数：`g_i2c1_mutex_error_count`、`g_sensor_queue_send_count`、`g_sensor_queue_drop_count`、`g_sht30_read_count`、`g_sht30_error_count`、`g_oled_mutex_retry_count`、`g_oled_update_error_count`、`g_comm_tx_ok_count`、`g_comm_tx_error_count`、`g_comm_mutex_error_count`、`g_comm_format_error_count`。

`g_mpu_data`、`g_mpu_physical_data`、`g_mpu_status`、`g_sht30_data`、`g_sht30_start_status`、`g_sht30_read_status`、`g_sensor_queue_status` 和 `g_process_message` 参与实际采集/状态判断/队列操作，不应仅因名称带 `g_` 就删除。后续可在确认全工程无 `extern` 引用后，再考虑局部化。

### 10.3 `hello_world_main.c`

- 删除未使用的 `text_buffer` 和已注释的原始 HEX/ASCII 调试输出代码。
- 删除重复的 `esp_err.h` include。
- 删除 `s_hello_published`、`MQTT_TOPIC` 和一次性 Hello 测试消息分支。
- 保留 UART 行接收、TLM 解析、JSON 组包、Wi-Fi 和 telemetry Topic 发布。
- 将“每秒最多发布一条”注释修正为“每 200 ms 最多发布一条”；不改当前 200000 μs 常量。
- 清理 STM32 `defaultTask` 中的一次性 USART3 测试字符串，避免 ESP32 收到非 TLM 文本。

全工程检查命令：

```bash
git grep -niE 'INA219|ina219|voltage_v|current_a|power_w|TELEMETRY_VALID_INA219|SENSOR_TYPE_INA219'
```

## 11. 后续升级路线

| 阶段 | 升级主题 | 主要工作 |
|---|---|---|
| V1.3 | 当前 DEMO 清理与冻结 | 删除 INA219 残留和无用调试定义；编译、烧录、OLED 回归；MQTTX 订阅验收 |
| V1.4 | UART 可靠性 | 协议版本、帧头、长度、序号、CRC16；异常帧丢弃；必要时 ACK/重传；逐步迁移 DMA/IRQ |
| V1.5 | ESP32 网络与云端 | MQTT TLS/认证、QoS1、设备独立 Topic、LWT、重连退避；ThingsBoard Telemetry/Attributes/Dashboard/Alarm/RPC |
| V1.6 | W25Q64 离线存储 | SPI2 驱动、StorageTask、循环日志/事件记录、网络恢复后按序补发 |
| V1.7 | 控制与系统健康 | ControlTask、Relay、Buzzer、PWM；IWDG、任务心跳、栈水位、堆余量、队列水位 |
| V2.0 | 工程化与长期测试 | 长时间压力测试、故障注入、参数配置、日志、版本发布、演示与测试报告 |

### 11.1 UART 协议升级建议

后续可升级为：

```text
AA 55 + VER + DEV + SEQ + TYPE + LEN + PAYLOAD + CRC16
```

CRC 覆盖 VER 至 PAYLOAD；SEQ 用于识别重复/丢帧；ACK/超时重传作为后续可靠性功能。不要在当前 DEMO 验收前同时改动协议与 MQTT 逻辑。

### 11.2 MQTT/云端升级建议

- 从公共明文 Broker 迁移到自有 Broker 或 ThingsBoard，配置 TLS 与设备认证。
- 按设备 ID 规划 Topic，避免多个设备共用固定 Topic。
- 根据业务要求采用 QoS1，并设计重复消息去重策略。
- 云端优先上传边缘摘要、振动状态和事件，不把 200 Hz 原始 MPU 数据全部推送到云端；需要波形时再做 Burst Upload。

### 11.3 存储、控制与看门狗

- W25Q64 通过 SPI2 接入，先完成读写/擦除/掉电恢复测试，再引入 StorageTask。
- Store-and-Forward 需处理缓存上限、Flash 磨损、满容量和掉电中断。
- ControlTask 加入本地安全条件、输出默认态和命令超时。
- WatchdogTask 监控关键任务心跳、栈高水位、堆余量、队列积压和关键错误计数。

## 12. DEMO 验收测试建议

| 测试 | 建议通过条件 |
|---|---|
| 传感器与 OLED | 三页面稳定；数据持续更新；无黑屏/卡页 |
| I2C 共享总线 30–60 min | mutex 错误计数保持 0；OLED 更新错误计数无持续增长 |
| 队列负载 | 队列丢帧计数不持续增长；队列不长期满载 |
| UART 连续传输 | TLM 字段数始终为 14；序号按预期递增；无持续乱码/超长帧 |
| MQTT 遥测 | JSON 可解析；单位/符号正确；序号递增；更新间隔符合当前 200 ms 限流 |
| Wi-Fi 断线恢复 | ESP32 恢复 MQTT 发布；STM32 采集和 OLED 不受影响 |
| 长时间运行 | 至少 1 h 无异常复位/任务卡死/堆耗尽；错误计数无异常增长 |

## 13. 当前已知限制与风险

- TLM 文本帧无 CRC/ACK/Retry；UART 解析依赖逗号分隔和换行边界。
- STM32 使用阻塞式 `HAL_UART_Transmit()`；后续评估 DMA/中断发送。
- ESP32 当前连接公共明文 MQTT Broker，QoS0，适合开发验证，不适合真实工业部署。
- `valid_mask` 主要表示数据曾经有效；后续增加时间戳新鲜度/超时判定。
- MPU6050 姿态精度受零偏、安装方向、温漂和算法限制；Yaw 无磁力计校正会漂移。
- 当前未接入 W25Q64、ThingsBoard、执行器控制、看门狗和 OTA。
- 本次审查只覆盖本轮上传的三个源文件；完整工程仍需全局搜索 INA219 残留并全量编译。

## 14. 跨会话恢复清单

必须：最新 `freertos.c`、`sensor_data.h`、`hello_world_main.c`。  
强烈建议：`mpu6050.c/.h`、`sht30.c/.h`、`oled.c/.h`、`main.c`、`usart.c/.h`、当前 CubeMX `.ioc`。  
通信联调：STM32 原始 TLM 日志、ESP32 串口日志、MQTTX 订阅 JSON 截图或导出。  
硬件：实际核心板原理图、STM32↔ESP32 接线照片、传感器和 OLED 接线记录。

## 15. 本版本结论

V1.3 将项目状态从 V1.2 的“STM32 本地采集与 OLED HMI 基线”更新为“STM32 采集/边缘处理/本地 HMI + CommTask 文本遥测 + ESP32-S3 UART/Wi-Fi/MQTT 源码链路已形成”的阶段性 DEMO/MVP。

当前最优先事项是完成 INA219 永久清理、删除无引用的临时调试变量、冻结现有 ASCII TLM 数据字段，并通过 MQTTX 和长时间运行测试形成可复现验收记录。之后按 V1.4→V2.0 路线升级通信可靠性、云端能力、离线存储、控制与系统健康管理。
