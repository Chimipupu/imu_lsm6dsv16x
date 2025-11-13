/**
 * @file drv_lsm6dsv16x.h
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief 6軸IMU LSM6DSV16Xドライバ
 * @version 0.1
 * @date 2025-11-13
 * 
 * @copyright Copyright (c) 2025 Chimipupu All Rights Reserved.
 * 
 */

#ifndef DRV_LSM6DSV16X_H
#define DRV_LSM6DSV16X_H

#include <stdint.h>

// [LSM6DSV16Xレジスタ]
typedef enum {
    REG_ADDR_FUNC_CFG_ACCESS = 0x01,
    REG_ADDR_PIN_CTRL,
    REG_ADDR_IF_CFG,
    // Reserved ... 0x04~0x05
    REG_ADDR_ODR_TRIG_CFG = 0x06,
    REG_ADDR_FIFO_CTRL_1,
    REG_ADDR_FIFO_CTRL_2,
    REG_ADDR_FIFO_CTRL_3,
    REG_ADDR_FIFO_CTRL_4,
    REG_ADDR_COUNTER_BDR_REG_1,
    REG_ADDR_COUNTER_BDR_REG_2,
    REG_ADDR_INT1_CTRL,
    REG_ADDR_INT2_CTRL,
    REG_ADDR_WHO_AM_I,
    REG_ADDR_CTRL_1,
    REG_ADDR_CTRL_2,
    REG_ADDR_CTRL_3,
    REG_ADDR_CTRL_4,
    REG_ADDR_CTRL_5,
    REG_ADDR_CTRL_6,
    REG_ADDR_CTRL_7,
    REG_ADDR_CTRL_8,
    REG_ADDR_CTRL_9,
    REG_ADDR_CTRL_10,
    REG_ADDR_CTRL_STATUS,
    REG_ADDR_FIFO_STATUS_1,
    REG_ADDR_FIFO_STATUS_2,
    REG_ADDR_ALL_INT_SRC,
    REG_ADDR_STATUS_REG,
    // Reserved ... 0x1F
    REG_ADDR_OUT_TEMP_L = 0x20,
    REG_ADDR_OUT_TEMP_H,
    REG_ADDR_OUTX_L_G,
    REG_ADDR_OUTX_H_G,
    REG_ADDR_OUTY_L_G,
    REG_ADDR_OUTY_H_G,
    REG_ADDR_OUTZ_L_G,
    REG_ADDR_OUTZ_H_G,
    REG_ADDR_OUTX_L_A,
    REG_ADDR_OUTX_H_A,
    REG_ADDR_OUTY_L_A,
    REG_ADDR_OUTY_H_A,
    REG_ADDR_OUTZ_L_A,
    REG_ADDR_OUTZ_H_A,
    REG_ADDR_UI_OUTX_L_G_OIS_EIS,
    REG_ADDR_UI_OUTX_H_G_OIS_EIS,
    REG_ADDR_UI_OUTY_L_G_OIS_EIS,
    REG_ADDR_UI_OUTY_H_G_OIS_EIS,
    REG_ADDR_UI_OUTZ_L_G_OIS_EIS,
    REG_ADDR_UI_OUTZ_H_G_OIS_EIS,
    REG_ADDR_UI_OUTX_L_A_OIS_DualC,
    REG_ADDR_UI_OUTX_H_A_OIS_DualC,
    REG_ADDR_UI_OUTY_L_A_OIS_DualC,
    REG_ADDR_UI_OUTY_H_A_OIS_DualC,
    REG_ADDR_UI_OUTZ_L_A_OIS_DualC,
    REG_ADDR_UI_OUTZ_H_A_OIS_DualC,
    REG_ADDR_AH_QVAR_OUT_L,
    REG_ADDR_AH_QVAR_OUT_H,
    // Reserved ... 0x3C~0x3F
    REG_ADDR_TIMESTAMP_0 = 0x40,
    REG_ADDR_TIMESTAMP_1,
    REG_ADDR_TIMESTAMP_2,
    REG_ADDR_TIMESTAMP_3,
    REG_ADDR_UI_STATUS_REG_OIS,
    REG_ADDR_WAKE_UP_SRC,
    REG_ADDR_TAP_SRC,
    REG_ADDR_D6D_SRC,
    REG_ADDR_STATUS_MASTER_MAINPAGE,
    REG_ADDR_EMB_FUNC_STATUS_MAINPAGE,
    REG_ADDR_FSM_STATUS_MAINPAGE,
    REG_ADDR_MLC_STATUS_MAINPAGE,
    // Reserved ... 0x4C~0x4E
    REG_ADDR_INTERNAL_FREQ_FINE = 0x4F,
    REG_ADDR_FUNCTIONS_ENABLE,
    REG_ADDR_DEN,
    // Reserved ... 0x52~0x53
    REG_ADDR_INACTIVITY_DUR = 0x54,
    REG_ADDR_INACTIVITY_THS,
    REG_ADDR_TAP_CFG_0,
    REG_ADDR_TAP_CFG_1,
    REG_ADDR_TAP_CFG_2,
    REG_ADDR_TAP_THS_6D,
    REG_ADDR_TAP_DUR,
    REG_ADDR_WAKE_UP_THS,
    REG_ADDR_WAKE_UP_DUR,
    REG_ADDR_FREE_FALL,
    REG_ADDR_MD1_CFG,
    REG_ADDR_MD2_CFG,
    // Reserved ... 0x60~0x61
    REG_ADDR_HAODR_CFG = 0x62,
    REG_ADDR_EMB_FUNC_CFG,
    REG_ADDR_UI_HANDSHAKE_CTRL,
    REG_ADDR_UI_SPI2_SHARED_0,
    REG_ADDR_UI_SPI2_SHARED_1,
    REG_ADDR_UI_SPI2_SHARED_2,
    REG_ADDR_UI_SPI2_SHARED_3,
    REG_ADDR_UI_SPI2_SHARED_4,
    REG_ADDR_UI_SPI2_SHARED_5,
    REG_ADDR_CTRL_EIS,
    // Reserved ... 0x6C~0x6E
    REG_ADDR_UI_INT_OIS = 0x6F,
    REG_ADDR_UI_CTRL1_OIS,
    REG_ADDR_UI_CTRL2_OIS,
    REG_ADDR_UI_CTRL3_OIS,
    REG_ADDR_X_OFS_USR,
    REG_ADDR_Y_OFS_USR,
    REG_ADDR_Z_OFS_USR,
    // Reserved ... 0x76~0x77
    REG_ADDR_FIFO_DATA_OUT_TAG = 0x78,
    REG_ADDR_FIFO_DATA_OUT_X_L,
    REG_ADDR_FIFO_DATA_OUT_X_H,
    REG_ADDR_FIFO_DATA_OUT_Y_L,
    REG_ADDR_FIFO_DATA_OUT_Y_H,
    REG_ADDR_FIFO_DATA_OUT_Z_L,
    REG_ADDR_FIFO_DATA_OUT_Z_H,
} E_LSM6DSV16X_REG_ADDR;

// FUNC_CFG_ACCESSレジスタ(Addr = 0x01)
typedef union {
    uint8_t BYTE;
    struct {
        uint8_t OIS_CTRL_FROM_UI:1;    // Bit 0 ... OIS_CTRL_FROM_UIビット
        uint8_t SPI2_RESET:1;          // Bit 1 ... SPI2_RESETビット
        uint8_t SW_POR:1;              // Bit 2 ... SW_PORビット
        uint8_t FSM_WR_CTRL_EN:1;      // Bit 3 ... FSM_WR_CTRL_ENビット
        uint8_t RESERVED:2;            // Bit4~5 ... Reserved
        uint8_t SHUB_REG_ACCESS:1;     // Bit 6 ... SHUB_REG_ACCESSビット
        uint8_t EMB_FUNC_REG_ACCESS:1; // Bit 7 ... EMB_FUNC_REG_ACCESSビット
    } BIT;
} REG_FUNC_CFG_ACCESS_T;

// PIN_CTRLレジスタ(Addr = 0x02)
typedef union {
    uint8_t BYTE;
    struct {
        uint8_t RESERVED:2;            // Bit[4:0] ... Reserved
        uint8_t IBHR_POR_EN:1;         // Bit 5 ... IBHR_POR_ENビット
        uint8_t SDO_PU_EN:1;           // Bit 6 ... SDO_PU_ENビット
        uint8_t OIS_PU_DIS:1;          // Bit 7 ... OIS_PU_DISビット
    } BIT;
} REG_PIN_CTRL_T;

// IF_CFGレジスタ(Addr = 0x03)
typedef union {
    uint8_t BYTE;
    struct {
        uint8_t I2C_I3C_DISABLE:1;     // Bit 0 ... I2C_I3C_disableビット
        uint8_t RESERVED:1;            // Bit 1 ... Reserved
        uint8_t SIM:1;                 // Bit 2 ... SIMビット
        uint8_t PP_OD:1;               // Bit 3 ... PP_ODビット
        uint8_t H_LACTIVE:1;           // Bit 4 ... H_LACTIVEビット
        uint8_t ASF_CTRL:1;            // Bit 5 ... ASF_CTRLビット
        uint8_t SHUB_PU_EN:1;          // Bit 6 ... SHUB_PU_ENビット
        uint8_t SDA_PU_EN:1;           // Bit 7 ... SDA_PU_ENビット
    } BIT;
} REG_IF_CFG_T;

void drv_lsm6dsv16x_init(void);

#endif // DRV_LSM6DSV16X_H