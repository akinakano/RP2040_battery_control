/**********************************************************************//**
* @file  imu_icm42688.h
* @brief imu icm42688と通信する関数を宣言する
*
* 開発部課：AIRBG SR事業室 駆動制御課
* 作成者：高木和貴/Shingo Nishikata(esOL)
* (C) 2022- Sony Corporation
*************************************************************************/

#ifndef __IMU_ICM42688_H
#define __IMU_ICM42688_H

#if (USE_CAMERA_IMU_TYPE == IMU_TYPE_ICM42688)

#include "stm32h7xx_hal.h"
#include "imu_common.h"
#include "stm32h7xx_hal_spi.h"

#define IMU_SPI      hspi1
#define IMU_DMAC_TX  hdma_spi1_tx
#define IMU_DMAC_RX  hdma_spi1_rx
#define IMU_SPI_CS   SPI1_CS

/* 換算用係数 */
#define IMU_FS_16g_COEFF            (4.882813E-01f)
#define IMU_FS_8g_COEFF             (2.441406E-01f)
#define IMU_FS_4g_COEFF             (1.220703E-01f)
#define IMU_FS_2g_COEFF             (6.103516E-02f)

#define IMU_FS_2000dps_COEFF        (6.103516E+01f)
#define IMU_FS_1000dps_COEFF        (3.051758E+01f)
#define IMU_FS_500dps_COEFF         (1.525879E+01f)
#define IMU_FS_250dps_COEFF         (7.629395E+00f)
#define IMU_FS_125dps_COEFF         (3.814697E+00f)
#define IMU_FS_62p5dps_COEFF        (1.907349E+00f)
#define IMU_FS_31p25dps_COEFF       (9.536743E-01f)
#define IMU_FS_15p625dps_COEFF      (4.768372E-01f)

/* dps->mdps->rad/s */
#define IMU_FROM_FS_TO_mRADS(lsb)   (float)((lsb) * IMU_FS_1000dps_COEFF * TO_RAD_COEFF * 0.001f)
#define IMU_FROM_FS_TO_mG(lsb)      (float)((lsb) * IMU_FS_16g_COEFF * TO_G_COEFF * 0.001f )

#define IMU_FROM_LSB_TO_degC(lsb)   (float)((lsb * 7.548309E-03f) + 25.0f)

/* Public Function */
void           icm42688_Initialize();
void           icm42688_Int(uint16_t imu_idx);
imu_float_data icm42688_GetDataFloat_blocking(SPI_HandleTypeDef *hspi);
imu_float_data icm42688_GetDataFloat(uint16_t imu_idx);
EN_IMU_ERR     icm42688_GetError(SPI_HandleTypeDef *hspi);

#endif

#endif // __IMU_ICM42688_H

