/**
 * @file app_imu_lsm6dsv16x.h
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief LSM6DSV16Xアプリヘッダー (for PicoSDK)
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025 Chimipupu All Rights Reserved.
 * 
 */

#ifndef APP_IMU_LSM6DSV16X_H
#define APP_IMU_LSM6DSV16X_H

// ---------------------------------------------------
// [コンパイルスイッチ]
#define SENSOR_PEDOMETER_USE // センサーのpedometer(歩数計)機能の使用有無

// ---------------------------------------------------
// [Define]

// ---------------------------------------------------
// [Prototype]
void app_lsm6dsv16x_main(void);
void app_lsm6dsv16x_init(void);

#endif // APP_IMU_LSM6DSV16X_H