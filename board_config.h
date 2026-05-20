#pragma once

// 板级硬件配置
// Board-level hardware configuration
// 通过条件编译自动区分 ESP32、ESP32-C3 与 ESP32-S3，无需手动修改

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ---------------------- ESP32C3 ------------------------- 

// ---- 电机引脚（数组顺序：RL, RR, FR, FL，与 motors.ino 中常量定义一致）----
// GPIO0/1/3/10 均无内置上拉，无 Strapping 功能（C3 的 Strapping 引脚仅 GPIO2/8/9）
#define BOARD_MOTOR_PINS   {3, 10, 0, 1}    // RL=GPIO3, RR=GPIO10, FR=GPIO0, FL=GPIO1

// ---- 电池 ADC ----
// GPIO2 为 Strapping 引脚（内置弱上拉），但 ADC 输入模式下上拉电阻不影响测量精度
// 启动后 Strapping 采样完成，analogRead() 调用时引脚为正常输入态
#define BOARD_VBAT_ADC_PIN 2               // ADC1_CH2（GPIO2）

// ---- RC 串口（SBUS）----
// GPIO9 有板载 I2C SCL 上拉 + Boot 内置上拉，作为 SBUS RX 输入时 SBUS 空闲高电平与上拉一致
#define BOARD_RC_SERIAL    Serial1
#define BOARD_RC_RX_PIN    9

// ---- IMU SPI（使用文档 / Arduino pins_arduino.h 默认引脚）----
// 与 Arduino ESP32-C3 库 SPI 默认定义完全一致，SPI.begin() 可无参调用
#define BOARD_SPI_SCK      4               // 文档 SCK  = GPIO4
#define BOARD_SPI_MISO     5               // 文档 MISO = GPIO5
#define BOARD_SPI_MOSI     6               // 文档 MOSI = GPIO6
#define BOARD_SPI_CS       7               // 文档 SS   = GPIO7

// ---- LED：无板载 LED，禁用 ----
#define BOARD_LED_ENABLED  0

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------- ESP32S3 -------------------------

// ---- 电机引脚：GPIO4-7，远离 FSPI 区（GPIO10-13），无 Strapping 约束 ----
#define BOARD_MOTOR_PINS   {4, 5, 6, 7}    // RL=GPIO4, RR=GPIO5, FR=GPIO6, FL=GPIO7

// ---- 电池 ADC：GPIO1（ADC1_CH0，S3 的 ADC1 范围为 GPIO1-10）----
#define BOARD_VBAT_ADC_PIN 1

// ---- RC 串口（SBUS）：GPIO8，与 FSPI/UART0 无冲突 ----
#define BOARD_RC_SERIAL    Serial2
#define BOARD_RC_RX_PIN    8

// ---- IMU SPI：使用 ESP32-S3 FSPI 默认管脚 ----
// SCK=12（默认），MISO=13（默认），MOSI=11（默认），CS=10（默认 SS）
#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10

// ---- LED：新 PCB 接 GPIO2 普通 LED，驱动方式与 ESP32 相同 ----
#define BOARD_LED_ENABLED  1

#else  // ---- ESP32 默认配置 ----
// ---------------------- 默认ESP32 -------------------------

#define BOARD_MOTOR_PINS   {12, 13, 15, 14}  // RL=GPIO12, RR=GPIO13, FR=GPIO15, FL=GPIO14
#define BOARD_VBAT_ADC_PIN 36
#define BOARD_RC_SERIAL    Serial2
#define BOARD_RC_RX_PIN    4
#define BOARD_SPI_SCK      18
#define BOARD_SPI_MISO     19
#define BOARD_SPI_MOSI     23
#define BOARD_SPI_CS       5
#define BOARD_LED_ENABLED  1

#endif
