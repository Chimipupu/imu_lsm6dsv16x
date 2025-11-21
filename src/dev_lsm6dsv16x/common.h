/**
 * @file common.h
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief 共通ヘッダー
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025 Chimipupu All Rights Reserved.
 * 
 */

#ifndef COMMON_H
#define COMMON_H
// ---------------------------------------------------
// [Include]

// C Std Lib
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// PicoSDK
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

// My Src Inc

// ---------------------------------------------------
// [コンパイルスイッチ]
// #define WDT_USE
// #define HW_PIO_USE

// ---------------------------------------------------
// [Define]

// GPIO
#define SENSOR_INT_PIN      20 // センサからの割り込みピン GPIO25

// I2C
#define I2C_PORT            i2c0
#if 1
#define I2C_SDA             4
#define I2C_SCL             5
#else
#define I2C_SDA             8
#define I2C_SCL             9
#endif

// SPI
#define SPI_PORT            spi0
#define PIN_MISO            16
#define PIN_CS              17
#define PIN_SCK             18
#define PIN_MOSI            19

// UART
#define UART_PORT           uart1
#define BAUD_RATE           115200
#define UART_TX_PIN         4
#define UART_RX_PIN         5

// ---------------------------------------------------
// [Global]

// ---------------------------------------------------
// [Static]
// ---------------------------------------------------
// [Func]

#endif // COMMON_H