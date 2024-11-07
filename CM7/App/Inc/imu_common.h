/**********************************************************************//**
* @file  imu_common.h
* @brief common definition for IMU_SPI interface
*
* 開発部課：AIRBG SR事業室 駆動制御課
* 作成者：高木和貴/Shingo.Nishikata
* (C) 2022 Sony Corporation
*************************************************************************/

#ifndef __IMU_COMMON_H
#define __IMU_COMMON_H

//#include "stm32g4xx_hal.h"
//#include "Define/commonType.h"
//#include "Define/apiDefine.h"
#include "stdbool.h"
#include "stm32h747xx.h"
#include "stm32h7xx_hal.h"
#include "math.h"

/* Private define ------------------------------------------------------------*/
#define IMU_DATA_WORD_N    (4)    /*!< IMU ICM-40602 受信データバッファ長. BANK切替のため1WORD追加 */
#define IMU_NUM_OF_MAX     (2)

#define IMU_ITEMSIZE       (8)                /*!< IMU Item Num             */
#define DMA_DATASIZE       (IMU_ITEMSIZE * 2) /*!< IMU DMAデータ長           */
#define DATA_BUFF_N        (2)  /*!< IMUデータのバッファ数(2面) */

/* 換算用係数 */
#define TO_RAD_COEFF       (3.14159265358979f / 180.0f)
#define TO_G_COEFF         (9.80665f)

/** IMU初期化シーケンスステータス */
typedef enum {
    EN_TSPI_STATUS_NON = 0,       /*!< 電源投入直後 */
    EN_TSPI_STATUS_INIT,          /*!< 初期化状態 */
    EN_TSPI_STATUS_DATA,           /*!< データ取得状態 */
} EN_TSPI_STATUS;

/* Private typedef -----------------------------------------------------------*/
/** IMU SPI 受信データ構造体 */
typedef union {
    unsigned long a_ul_word[IMU_DATA_WORD_N];   /*!< 4Byteアクセス */

    struct {
        int16_t    ss_temp_data;           /*!< 温度 */
        int16_t    :16;                    /*    */
        int16_t    ss_accel_data_y;        /*!< 加速度α y軸 Qs.15 */
        int16_t    ss_accel_data_x;        /*!< 加速度α x軸 Qs.15 */
        int16_t    ss_gyro_data_x;         /*!< 角速度ω x軸 Qs.15 */
        int16_t    ss_accel_data_z;        /*!< 加速度α z軸 Qs.15 */
        int16_t    ss_gyro_data_z;         /*!< 角速度ω z軸 Qs.15 */
        int16_t    ss_gyro_data_y;         /*!< 角速度ω y軸 Qs.15 */
    }st_data;                              /*!< データアクセス */
}UN_IMU_DATA;

//IMUのエラー値
typedef enum {
    EN_IMU_ERR_INI = 0xFF, /* 初期化       */
    EN_IMU_ERR_NON = 0,    /* エラーなし   */
    EN_IMU_ERR_WHOAMI,     /* WhoAmIエラー */
    EN_IMU_ERR_RESET,      /* RESETエラー  */
} EN_IMU_ERR;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *hcs_port;
    uint16_t          cs_pin;
} ST_IMU_SPI;

typedef struct {
    SPI_HandleTypeDef *hspi;
    bool transmit_flag;
    uint8_t  data_buff_no;
    uint8_t data_Buff[DATA_BUFF_N][DMA_DATASIZE];
} ST_DMA_INFO;

typedef struct {
    uint32_t    timestamp;
    float temperature;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;
} imu_float_t;

typedef struct {
    uint32_t timestamp;
    int32_t temperature;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
} imu_int_t;


typedef struct {
    float temp_data;

    float angular_rate_mrads_x;
    float angular_rate_mrads_y;
    float angular_rate_mrads_z;

    float acceleration_mg_x;
    float acceleration_mg_y;
    float acceleration_mg_z;
} imu_float_data;


#endif // __IMU_COMMON_H

