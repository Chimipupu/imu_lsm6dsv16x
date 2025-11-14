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
static uint32_t s_step = 0;

#if defined(SENSOR_PEDOMETER_USE)
static uint8_t s_step_event_catched = 0;
#else
static lsm6dsv16x_filt_settling_mask_t s_filt_settling_mask;
static int16_t s_raw_acceleration_buf[3];
static int16_t s_raw_angular_rate_buf[3];
static int16_t s_raw_temperature;
static double_t s_acceleration_mg_buf[3];
static double_t s_angular_rate_mdps_buf[3];
static double_t s_temperature_degC;
#endif // SENSOR_PEDOMETER_USE

// ---------------------------------------------------
// [Prototype]
static int32_t platform_write(void *p_handle, uint8_t reg, const uint8_t *p_buf, uint16_t len);
static int32_t platform_read(void *p_handle, uint8_t reg, uint8_t *p_buf, uint16_t len);
static void platform_delay(uint32_t ms);

#if defined(SENSOR_PEDOMETER_USE)
static void sensor_pedometer_init(void);
#else
static void sensor_init(void);
#endif // SENSOR_PEDOMETER_USE

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

static void sensor_init(void)
{
    lsm6dsv16x_xl_data_rate_set(&s_drv_ctx, LSM6DSV16X_ODR_AT_7Hz5);
    lsm6dsv16x_gy_data_rate_set(&s_drv_ctx, LSM6DSV16X_ODR_AT_15Hz);

    lsm6dsv16x_xl_full_scale_set(&s_drv_ctx, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&s_drv_ctx, LSM6DSV16X_2000dps);

    s_filt_settling_mask.drdy = PROPERTY_ENABLE;
    s_filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    s_filt_settling_mask.irq_g = PROPERTY_ENABLE;

    lsm6dsv16x_filt_settling_mask_set(&s_drv_ctx, s_filt_settling_mask);
    lsm6dsv16x_filt_gy_lp1_set(&s_drv_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&s_drv_ctx, LSM6DSV16X_GY_ULTRA_LIGHT);
    lsm6dsv16x_filt_xl_lp2_set(&s_drv_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&s_drv_ctx, LSM6DSV16X_XL_STRONG);
}

#if defined(SENSOR_PEDOMETER_USE)
static void sensor_pedometer_init(void)
{
    // センサーのpedometer(歩数計)機能を有効化
    lsm6dsv16x_stpcnt_mode_t stpcnt_mode = { 0 };
    stpcnt_mode.step_counter_enable = PROPERTY_ENABLE;
    stpcnt_mode.false_step_rej = PROPERTY_ENABLE;
    lsm6dsv16x_stpcnt_mode_set(&s_drv_ctx, stpcnt_mode);
    pin_int.step_det = PROPERTY_ENABLE;

    // センサー外部割り込みピン設定
    lsm6dsv16x_emb_pin_int1_route_set(&s_drv_ctx, &pin_int);
    //lsm6dsv16x_emb_pin_int2_route_set(&s_drv_ctx, &pin_int);
    lsm6dsv16x_embedded_int_cfg_set(&s_drv_ctx, LSM6DSV16X_INT_LATCH_ENABLE);

    // センサー出力レート設定
    lsm6dsv16x_xl_data_rate_set(&s_drv_ctx, LSM6DSV16X_ODR_AT_120Hz);

    // センサースケール設定
    lsm6dsv16x_xl_full_scale_set(&s_drv_ctx, LSM6DSV16X_2g);
}
#endif // SENSOR_PEDOMETER_USE

// ---------------------------------------------------
// [Global関数]

#if defined(SENSOR_PEDOMETER_USE)
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
#endif // SENSOR_PEDOMETER_USE

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

#if defined(SENSOR_PEDOMETER_USE)
    // センサーから歩数をReadできる設定で初期化
    sensor_pedometer_init();
#else
    // センサーから生データをReadできる設定で初期化
    sensor_init();
#endif // SENSOR_PEDOMETER_USE
}

/**
 * @brief LSM6DSV16Xアプリメイン
 * 
 */
void app_lsm6dsv16x_main(void)
{
    // [歩数計アプリ]
#if defined(SENSOR_PEDOMETER_USE)
    uint16_t steps;
    if (s_step_event_catched) {
        s_step_event_catched = 0;
        lsm6dsv16x_stpcnt_steps_get(&s_drv_ctx, &steps);
        s_step += (uint32_t)steps;
        printf("Sensor Pedometer Steps :%d\r\n", s_step);
    }
#else
    // [センサーの生データのRead]
    lsm6dsv16x_data_ready_t drdy;
    lsm6dsv16x_flag_data_ready_get(&s_drv_ctx, &drdy);

    // [加速度の読み出し]
    if (drdy.drdy_xl) {
        memset(s_raw_acceleration_buf, 0x00, 3 * sizeof(int16_t));
        lsm6dsv16x_acceleration_raw_get(&s_drv_ctx, s_raw_acceleration_buf);
        s_acceleration_mg_buf[0] =
            lsm6dsv16x_from_fs2_to_mg(s_raw_acceleration_buf[0]);
        s_acceleration_mg_buf[1] =
            lsm6dsv16x_from_fs2_to_mg(s_raw_acceleration_buf[1]);
        s_acceleration_mg_buf[2] =
            lsm6dsv16x_from_fs2_to_mg(s_raw_acceleration_buf[2]);
        printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                s_acceleration_mg_buf[0], s_acceleration_mg_buf[1], s_acceleration_mg_buf[2]);
    }

    // [角速度の読み出し]
    if (drdy.drdy_gy) {
        memset(s_raw_angular_rate_buf, 0x00, 3 * sizeof(int16_t));
        lsm6dsv16x_angular_rate_raw_get(&s_drv_ctx, s_raw_angular_rate_buf);
        s_angular_rate_mdps_buf[0] =
            lsm6dsv16x_from_fs2000_to_mdps(s_raw_angular_rate_buf[0]);
        s_angular_rate_mdps_buf[1] =
            lsm6dsv16x_from_fs2000_to_mdps(s_raw_angular_rate_buf[1]);
        s_angular_rate_mdps_buf[2] =
            lsm6dsv16x_from_fs2000_to_mdps(s_raw_angular_rate_buf[2]);
        printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                s_angular_rate_mdps_buf[0], s_angular_rate_mdps_buf[1], s_angular_rate_mdps_buf[2]);
    }

    // [温度の読み出し]
    if (drdy.drdy_temp) {
        memset(&s_raw_temperature, 0x00, sizeof(int16_t));
        lsm6dsv16x_temperature_raw_get(&s_drv_ctx, &s_raw_temperature);
        s_temperature_degC = lsm6dsv16x_from_lsb_to_celsius(
                            s_raw_temperature);
        printf("Temperature [degC]:%6.2f\r\n", s_temperature_degC);
        }
#endif // SENSOR_PEDOMETER_USE
}