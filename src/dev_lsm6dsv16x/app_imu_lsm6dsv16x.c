/**
 * @file app_imu_lsm6dsv16x.c
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief LSM6DSV16Xアプリ (for PicoSDK)
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025 Chimipupu All Rights Reserved.
 * 
 */

#include "app_imu_lsm6dsv16x.h"

// ---------------------------------------------------
// [Include]

// C Std Lib
#include <string.h>
#include <stdint.h>

// PicoSDK
#include "hardware/i2c.h"
#include "pico/time.h"

// ST Driver
#include "lsm6dsv16x_reg.h"

// My Src Inc
#include "common.h"

// ---------------------------------------------------
// [Define]

// ---------------------------------------------------
// [Global]

// ---------------------------------------------------
// [Static]
static stmdev_ctx_t s_drv_ctx;
static uint8_t s_who_am_i = 0;
static uint8_t s_step_event_catched = 0;
static uint32_t s_step = 0;

// ---------------------------------------------------
// [Prototype]
static int32_t platform_write(void *p_handle, uint8_t reg, const uint8_t *p_buf, uint16_t len);
static int32_t platform_read(void *p_handle, uint8_t reg, uint8_t *p_buf, uint16_t len);
static void platform_delay(uint32_t ms);

// ---------------------------------------------------
// [Static関数]
static int32_t platform_write(void *p_handle, uint8_t reg, const uint8_t *p_buf, uint16_t len)
{
    i2c_write_blocking(p_handle, reg, p_buf, len, false);
}

static int32_t platform_read(void *p_handle, uint8_t reg, uint8_t *p_buf, uint16_t len)
{
    i2c_read_blocking(p_handle, reg, p_buf, len, false);
}

static void platform_delay(uint32_t ms)
{
    sleep_ms(ms);
}


// ---------------------------------------------------
// [Global関数]

/**
 * @brief LSM6DSV16XのINT割り込みハンドラ
 * 
 */
void lsm6dsv16x_pedometer_handler(void)
{
    lsm6dsv16x_embedded_status_t status;

    lsm6dsv16x_embedded_status_get(&s_drv_ctx, &status);

    if (status.step_detector) {
        s_step_event_catched = 1;
    }
}

/**
 * @brief LSM6DSV16Xアプリ初期化
 * 
 */
void app_lsm6dsv16x_init(void)
{
    lsm6dsv16x_emb_pin_int_route_t pin_int = { 0 };

    // ドライバーにコールバック関数を渡す
    s_drv_ctx.write_reg = platform_write;
    s_drv_ctx.read_reg = platform_read;
    s_drv_ctx.mdelay = platform_delay;
    s_drv_ctx.handle = I2C_PORT;

#if 1
    // センサーの接続確認
    lsm6dsv16x_device_id_get(&s_drv_ctx, &s_who_am_i);
    if (s_who_am_i != LSM6DSV16X_ID) {
        while (1);
    }
#endif

    // センサー初期化
    lsm6dsv16x_sw_por(&s_drv_ctx);

    // センサーブロックデータ出力設定
    lsm6dsv16x_block_data_update_set(&s_drv_ctx, PROPERTY_ENABLE);

#ifdef SENSOR_PEDOMETER_USE
    // センサーのpedometer(歩数計)機能を有効化
    lsm6dsv16x_stpcnt_mode_t stpcnt_mode = { 0 };
    stpcnt_mode.step_counter_enable = PROPERTY_ENABLE;
    stpcnt_mode.false_step_rej = PROPERTY_ENABLE;
    lsm6dsv16x_stpcnt_mode_set(&s_drv_ctx, stpcnt_mode);
    pin_int.step_det = PROPERTY_ENABLE;
#endif // SENSOR_PEDOMETER_USE

    // センサー外部割り込みピン設定
    lsm6dsv16x_emb_pin_int1_route_set(&s_drv_ctx, &pin_int);
    //lsm6dsv16x_emb_pin_int2_route_set(&s_drv_ctx, &pin_int);
    lsm6dsv16x_embedded_int_cfg_set(&s_drv_ctx, LSM6DSV16X_INT_LATCH_ENABLE);

    // センサー出力レート設定
    lsm6dsv16x_xl_data_rate_set(&s_drv_ctx, LSM6DSV16X_ODR_AT_120Hz);
    // センサースケール設定
    lsm6dsv16x_xl_full_scale_set(&s_drv_ctx, LSM6DSV16X_2g);
}

/**
 * @brief LSM6DSV16Xアプリメイン
 * 
 */
void app_lsm6dsv16x_main(void)
{
    // 歩数計アプリ
#ifdef SENSOR_PEDOMETER_USE
    uint16_t steps;
    if (s_step_event_catched) {
        s_step_event_catched = 0;
        lsm6dsv16x_stpcnt_steps_get(&s_drv_ctx, &steps);
        s_step += (uint32_t)steps;
        printf("Sensor Pedometer Steps :%d\r\n", s_step);
    }
#endif // SENSOR_PEDOMETER_USE
}