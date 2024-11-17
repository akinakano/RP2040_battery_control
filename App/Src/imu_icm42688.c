/**********************************************************************//**
* @file  imu_icm42688.c
* @brief imu icm42688と通信する関数を定義する
*
* 開発部課：AIRBG SR事業室 駆動制御課
* 作成者：高木和貴/Shingo.Nishikata(esOL)
* (C) 2022- Sony Corporation
*************************************************************************/
#include <Math.h>
#include <limits.h>
#include <stdio.h>
#include <stdbool.h>
#include "imu_icm42688.h"
#include "main.h"
#include "HW_type.h"

#define ICM_ACC_FSR            (16)      /* +/-[g]   */
#define ICM_GYR_FSR            (1000)    /* +/-[dps] */

/*------------------------------------------------------------------------*/
/* デバッグ設定                                                           */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* IMU設定値                                                                 */
/*------------------------------------------------------------------------*/
#define C_US_IMU_WRITE       (0x00)          /*!<  */
#define C_US_IMU_READ        (0x80)          /*!<  */
#define C_UL_IMU_WRITE       (0x0000)        /*!<  */
#define C_UL_IMU_READ        (0x8000)        /*!<  */

/** IMU ICM-42688 初期設定通信(UI側) */
static const uint8_t SCA_US_IMU_INIT_SEND_UI[][2] = {
    {(C_US_IMU_WRITE  | 0x4C), 0x33},     /* INTF_CONFIG0 Disable I2C */

#if   ICM_GYR_FSR == 2000
    {(C_US_IMU_WRITE  | 0x4F), 0x03},     /* GYRO_CONFIG0 Gyro_FS:+/-2000dps Gyro_ODR:8kHz*/
#elif ICM_GYR_FSR == 1000
    {(C_US_IMU_WRITE  | 0x4F), 0x23},     /* GYRO_CONFIG0 Gyro_FS:+/-1000dps Gyro_ODR:8kHz*/
//    {(C_US_IMU_WRITE  | 0x4F), 0x24},     /* GYRO_CONFIG0 Gyro_FS:+/-1000dps Gyro_ODR:4kHz*/
//    {(C_US_IMU_WRITE  | 0x4F), 0x25},     /* GYRO_CONFIG0 Gyro_FS:+/-1000dps Gyro_ODR:2kHz*/
//    {(C_US_IMU_WRITE  | 0x4F), 0x26},     /* GYRO_CONFIG0 Gyro_FS:+/-1000dps Gyro_ODR:1kHz*/
#elif ICM_GYR_FSR ==  500
    {(C_US_IMU_WRITE  | 0x4F), 0x43},    /* GYRO_CONFIG0 Gyro_FS:+/-0500dps Gyro_ODR:8kHz*/
#else
    err
#endif

#if   ICM_ACC_FSR == 16
    {(C_US_IMU_WRITE  | 0x50), 0x03},    /* ACCEL_CONFIG0 Accel_FS:+/-16G Accel_ODR:8kHz*/
//    {(C_US_IMU_WRITE  | 0x50), 0x04},    /* ACCEL_CONFIG0 Accel_FS:+/-16G Accel_ODR:4kHz*/
//    {(C_US_IMU_WRITE  | 0x50), 0x05},    /* ACCEL_CONFIG0 Accel_FS:+/-16G Accel_ODR:2kHz*/
//    {(C_US_IMU_WRITE  | 0x50), 0x06},    /* ACCEL_CONFIG0 Accel_FS:+/-16G Accel_ODR:1kHz*/
#elif ICM_ACC_FSR ==  4
    {(C_US_IMU_WRITE  | 0x50), 0x46},    /* ACCEL_CONFIG0 Accel_FS:+/ -4G Accel_ODR:1kHz*/
#elif ICM_ACC_FSR ==  2
    {(C_US_IMU_WRITE  | 0x50), 0x66},    /* ACCEL_CONFIG0 Accel_FS:+/ -2G Accel_ODR:1kHz*/
#else
    err
#endif

    {(C_US_IMU_WRITE  | 0x51), 0x02},     /* GYRO_CONFIG1 Temp_FLT_BW:4000Hz Gyro_UI_FLT_ORD:1st GYRO_DEC2_M2_ORD:3rd */
    {(C_US_IMU_WRITE  | 0x52), 0x77},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 2096.30Hz Gyro_UI_FLT_BW: 2096.30Hz for ODR=8KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0xFF},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 2096.30Hz Gyro_UI_FLT_BW: 2096.30Hz for 1KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0x00},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 492.90Hz Gyro_UI_FLT_BW: 498.30Hz for 1KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0x11},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 234.70Hz Gyro_UI_FLT_BW: 227.20Hz for 1KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0x33},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 118.90Hz Gyro_UI_FLT_BW: 110.00Hz for 1KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0x55},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 60.80Hz Gyro_UI_FLT_BW: 59.60Hz for 1KHz*/
//    {(C_US_IMU_WRITE  | 0x52), 0x77},     /* GYRO_ACCEL_CONFIG0 Accel_UI_FLT_BW: 25.20Hz Gyro_UI_FLT_BW: 23.90Hz for 1KHz*/

    {(C_US_IMU_WRITE  | 0x53), 0x00},     /* ACCEL_CONFIG1 Accel_UI_FLT_ORD:3rd*/
    {(C_US_IMU_WRITE  | 0x4E), 0x0F},     /* PWR_MGMT0 Temp_Dis: Gyro_Mode: Accel_Mode:*/

//TEST:AAF
#if 0
    // AAF: bypass
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0B), 0x02},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DIS=Disable GYRO_NF_DIS=Enable*/
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x03},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=1 ACCEL_AAF_DIS=Disable */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
#if 0
    // AAF: 1766Hz: DELT=34 DELTSQR=1376(0x0560) BIT_SHIFT=4
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0C), 0x22},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DELT=34 */
    {(C_US_IMU_WRITE  | 0x0D), 0x60},     /* GYRO_CONFIG_STATIC3: GYRO_AAF_DELTSQR[7:0]=0x60 */
    {(C_US_IMU_WRITE  | 0x0E), 0x45},     /* GYRO_CONFIG_STATIC4: GYRO_AAF_BIT_SHIFT=4 GYRO_AAF_DELTSQR[11:8]=0x05 */
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x44},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=34 ACCEL_AAF_DIS=Enable */
    {(C_US_IMU_WRITE  | 0x04), 0x60},     /* ACCEL_CONFIG_STATIC3: ACCEL_AAF_DELTSQR[7:0]=0x60 */
    {(C_US_IMU_WRITE  | 0x05), 0x45},     /* ACCEL_CONFIG_STATIC4: ACCEL_AAF_BIT_SHIFT=4 ACCEL_AAF_DELTSQR[11:8]=0x5 */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
#if 0
    // AAF: 785Hz: DELT=17 DELTSQR=288(0x0120) BIT_SHIFT=7
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0C), 0x11},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DELT=17 */
    {(C_US_IMU_WRITE  | 0x0D), 0x20},     /* GYRO_CONFIG_STATIC3: GYRO_AAF_DELTSQR[7:0]=0x20 */
    {(C_US_IMU_WRITE  | 0x0E), 0x71},     /* GYRO_CONFIG_STATIC4: GYRO_AAF_BIT_SHIFT=7 GYRO_AAF_DELTSQR[11:8]=0x01 */
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x22},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=17 ACCEL_AAF_DIS=Enable */
    {(C_US_IMU_WRITE  | 0x04), 0x20},     /* ACCEL_CONFIG_STATIC3: ACCEL_AAF_DELTSQR[7:0]=0x20 */
    {(C_US_IMU_WRITE  | 0x05), 0x71},     /* ACCEL_CONFIG_STATIC4: ACCEL_AAF_BIT_SHIFT=7 ACCEL_AAF_DELTSQR[11:8]=0x1 */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
#if 0
    // AAF: 585Hz: DELT=13 DELTSQR=170(0x00AA) BIT_SHIFT=8
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0C), 0x0D},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DELT=13 */
    {(C_US_IMU_WRITE  | 0x0D), 0xAA},     /* GYRO_CONFIG_STATIC3: GYRO_AAF_DELTSQR[7:0]=0xAA */
    {(C_US_IMU_WRITE  | 0x0E), 0x80},     /* GYRO_CONFIG_STATIC4: GYRO_AAF_BIT_SHIFT=8 GYRO_AAF_DELTSQR[11:8]=0x00 */
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x1A},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=13 ACCEL_AAF_DIS=Enable */
    {(C_US_IMU_WRITE  | 0x04), 0xAA},     /* ACCEL_CONFIG_STATIC3: ACCEL_AAF_DELTSQR[7:0]=0xAA */
    {(C_US_IMU_WRITE  | 0x05), 0x80},     /* ACCEL_CONFIG_STATIC4: ACCEL_AAF_BIT_SHIFT=8 ACCEL_AAF_DELTSQR[11:8]=0x0 */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
#if 0
    // AAF: 394Hz: DELT=9 DELTSQR=81(0x0051) BIT_SHIFT=9
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0C), 0x09},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DELT=9 */
    {(C_US_IMU_WRITE  | 0x0D), 0x51},     /* GYRO_CONFIG_STATIC3: GYRO_AAF_DELTSQR[7:0]=0x51 */
    {(C_US_IMU_WRITE  | 0x0E), 0x90},     /* GYRO_CONFIG_STATIC4: GYRO_AAF_BIT_SHIFT=9 GYRO_AAF_DELTSQR[11:8]=0x0 */
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x12},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=17 ACCEL_AAF_DIS=Enable */
    {(C_US_IMU_WRITE  | 0x04), 0x51},     /* ACCEL_CONFIG_STATIC3: ACCEL_AAF_DELTSQR[7:0]=0x51 */
    {(C_US_IMU_WRITE  | 0x05), 0x90},     /* ACCEL_CONFIG_STATIC4: ACCEL_AAF_BIT_SHIFT=9 ACCEL_AAF_DELTSQR[11:8]=0x0 */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
#if 0
    // AAF: 42Hz: DELT=1 DELTSQR=1 BIT_SHIFT=15
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x0C), 0x01},     /* GYRO_CONFIG_STATIC2: GYRO_AAF_DELT=1 */
    {(C_US_IMU_WRITE  | 0x0D), 0x01},     /* GYRO_CONFIG_STATIC3: GYRO_AAF_DELTSQR[7:0]=0x01 */
    {(C_US_IMU_WRITE  | 0x0E), 0xF0},     /* GYRO_CONFIG_STATIC4: GYRO_AAF_BIT_SHIFT=15 GYRO_AAF_DELTSQR[11:8]=0x0 */
    {(C_US_IMU_WRITE  | 0x76), 0x02},     /* REG_BANK_SEL  ACCESS BANK2 */
    {(C_US_IMU_WRITE  | 0x03), 0x02},     /* ACCEL_CONFIG_STATIC2: ACCEL_AAF_DELT=1 ACCEL_AAF_DIS=Enable */
    {(C_US_IMU_WRITE  | 0x04), 0x01},     /* ACCEL_CONFIG_STATIC3: ACCEL_AAF_DELTSQR[7:0]=0x01 */
    {(C_US_IMU_WRITE  | 0x05), 0xF0},     /* ACCEL_CONFIG_STATIC4: ACCEL_AAF_BIT_SHIFT=15 ACCEL_AAF_DELTSQR[11:8]=0x0 */
    {(C_US_IMU_WRITE  | 0x76), 0x00},     /* REG_BANK_SEL  ACCESS BANK0 */
#endif
//    {(C_US_IMU_WRITE  | 0x14), 0x03},     /* INT_CONFIG INT1: Pulse mode, Push pull, Active high */
//    {(C_US_IMU_WRITE  | 0x64), 0x70},     /* INT_CONFIG1 Pulse duration: 8us */
//    {(C_US_IMU_WRITE  | 0x65), 0x18},     /* INT_SOURCE0 UI_DRDY_INT1_EN */

    /* ここからAUX1の設定 */
//    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
//    {(C_US_IMU_WRITE  | 0x13), 0x00},     /* GYRO_CONFIG_STATIC10 HPF_ORD:1st */
//    {(C_US_IMU_WRITE  | 0x7A), 0x06},     /* INTF_CONFIG4 AUZ1 use 4-wire SPI, AP use 4-wire SPI */
//    {(C_US_IMU_WRITE  | 0x76), 0x00}      /* REG_BANK_SEL  ACCESS BANK0 */
};

/** IMU ICM-42688 4線SPI設定通信(UI側) */
static const uint8_t SCA_US_IMU_SPI4_SEND_UI[][2] = {
    {(C_US_IMU_WRITE  | 0x76), 0x01},     /* REG_BANK_SEL  ACCESS BANK1 */
    {(C_US_IMU_WRITE  | 0x7A), 0x06},     /* INTF_CONFIG4 AUX1 use 4-wire SPI, AP use 4-wire SPI */
    {(C_US_IMU_WRITE  | 0x76), 0x00}      /* REG_BANK_SEL  ACCESS BANK0 */
};

/** IMU ICM-42688 データ取得通信(UI) */
//static const unsigned long SCA_UL_IMU_DATA_SEND_UI[4] = { C_UL_IMU_READ | 0x1C000000, 0x00000000, 0x00000000, 0x00000000 };

#define IMU_WHOAMI_ADDRESS            (0x75)
#define IMU_WHOAMI_RESPONSE           (0x47)

#define IMU_RESET_ADDRESS             (0x11)
#define IMU_RESET_COMMAND             (0x01)

#define IMU_RESET_DONE_ADDRESS        (0x2D)
#define IMU_RESET_DONE_MASK           (0x10)
#define IMU_RESET_DONE_FLAG           (0x10)

#define IMU_ACC_ADDR                  (0x1d)

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* Private variables ---------------------------------------------------------*/
static ST_IMU_SPI imu_spi_info[IMU_NUM_OF_MAX] = 
    {
        {NULL, NULL, 0},
        {NULL, NULL, 0}
    };

static ST_DMA_INFO dma_info[IMU_NUM_OF_MAX];
static EN_IMU_ERR s_en_imu_err[IMU_NUM_OF_MAX] = {EN_IMU_ERR_INI, EN_IMU_ERR_INI};     //IMUエラーステータス
static const uint16_t IMU_DATA_LENGTH = 14;
static uint8_t s_imu_read_transmit_data[16];

/*------------------------------------------------------------------------*/
/* IMUエラー処理                                                          */
/*------------------------------------------------------------------------*/
//IMUのリセット待ち期間。単位は通信回数。待ち期間が過ぎてもリセットが解除されない場合は、エラー扱いとする。
//IMUでaddress+dataの2byteを転送するには、(16bit +2) / ボーレート = ex.8.65us
//8.65us * IMU_RST_WAITだけ待てることになる。
#define IMU_RST_WAIT    (5)

/*------------------------------------------------------------------------*/
/* static interfaces                                                      */
/*------------------------------------------------------------------------*/

#if 0
/**************************************************************************
 * @author  Masayuki.Fukuyama
 * @date    20220310
 * @param   l0_time_ns : wait time 10nsec
 * @return  None
 * @brief   Waiting for 10nsec during operation at 170MHz
 *          1cycle : 0.000000005x = 1 / 170000000 
 * @warning
 * @detail
 *************************************************************************/
static void imu_spi_Delay_10nsec(int l0_time_ns)
{
    while( l0_time_ns > 0){
        asm("NOP"); asm("NOP");
        l0_time_ns -= 1;
    }
}
#endif

/**************************************************************************
 * @author  Masayuki.Fukuyama
 * @date    20220310
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @return  None
 * @brief   Turn Off chip selector for SPI communication for imu
 * @warning
 * @detail
 *************************************************************************/
static void imu_spi_ChipSelector_Off(ST_IMU_SPI *pSpiInfo)
{
    // Chip Selector OFF 
    HAL_GPIO_WritePin(pSpiInfo->hcs_port, pSpiInfo->cs_pin, GPIO_PIN_RESET); 

    // Wait 40 nanoseconds. 
    //  imu_spi_Delay_10nsec(4);
}

/**************************************************************************
 * @author  Masayuki.Fukuyama
 * @date    20220310
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @return  None
 * @brief   Turn On chip selector for SPI communication for imu
 * @warning
 * @detail
 *************************************************************************/
static void imu_spi_ChipSelector_On(ST_IMU_SPI *pSpiInfo)
{
    // Chip Selector ON 
    HAL_GPIO_WritePin(pSpiInfo->hcs_port, pSpiInfo->cs_pin, GPIO_PIN_SET); 

    // Wait 20 nanoseconds. 
    //  imu_spi_Delay_10nsec(2);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @return  void
 * @brief   SPI シングル送信関数
 * @warning フレーム長=16bitが事前設定されていることを期待する。16bit以外では正常動作しない。
 *************************************************************************/
static void Sv_TSPI_Write_Single(ST_IMU_SPI *pSpiInfo, uint8_t Reg_Addr, uint8_t Reg_value)
{
    uint8_t send_cmd[2] = {Reg_Addr, Reg_value};

    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(pSpiInfo);

    HAL_SPI_Transmit(pSpiInfo->hspi, send_cmd, 2, 10);

    // Chip Selector ON 
    imu_spi_ChipSelector_On(pSpiInfo);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @param   register address
 * @return  void
 * @brief   SPI シングル受信関数
 * @warning フレーム長=16bitが事前設定されていることを期待する。16bit以外では正常動作しない。
 * @warning 受信FIFOが空になっていることを期待する。受信FIFOはクリアされる。
 *************************************************************************/
static uint8_t Sv_TSPI_Read_Single(ST_IMU_SPI *pSpiInfo, uint8_t Reg_Addr)
{
    uint8_t send_cmd;
    uint8_t recv_data[2];

    send_cmd = C_US_IMU_READ | Reg_Addr;

    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(pSpiInfo);

    HAL_SPI_TransmitReceive(pSpiInfo->hspi, (uint8_t *)&send_cmd, (uint8_t *)&recv_data, 2, 10);

    // Chip Selector ON 
    imu_spi_ChipSelector_On(pSpiInfo);

//  printf("  [Debug]  HAL_SPI_TransmitReceive Read Data : 0x%02x/0x%02x .\r\n",
//           recv_data[0], recv_data[1]);
    return recv_data[1];
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @return  None
 * @brief   IMU 初期化通信用関数
 * @warning
 *************************************************************************/
static void IMU_SPI_Send_Initial_Settings(ST_IMU_SPI *pSpiInfo)
{
    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(pSpiInfo);

    /* 初期化設定を送る*/
    HAL_SPI_Transmit(pSpiInfo->hspi, (uint8_t *)SCA_US_IMU_INIT_SEND_UI, 
            sizeof(SCA_US_IMU_INIT_SEND_UI) / sizeof(SCA_US_IMU_INIT_SEND_UI[0][0]), 10);

    // Chip Selector ON 
    imu_spi_ChipSelector_On(pSpiInfo);
}


/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%190128
 * @param   pSpiInfo : Spi information table pointer for Imu
 * @return  None
 * @brief   IMUのSPIをUI側もAUX側も4線式SPIに設定する
 * @detail  SW-RSTの前のコマンド発効、RST後のコマンド発効両方の前に実行が必要
 *************************************************************************/
static void IMU_SPI_Chip_Set_SPI4(ST_IMU_SPI *pSpiInfo)
{
    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(pSpiInfo);

    HAL_SPI_Transmit(pSpiInfo->hspi, (uint8_t *)SCA_US_IMU_SPI4_SEND_UI, 
            sizeof(SCA_US_IMU_SPI4_SEND_UI), 10);

    // Chip Selector ON 
    imu_spi_ChipSelector_On(pSpiInfo);

//  printf("  [Debug]  HAL_SPI_Transmit ret : 0x%02x .\r\n", hal_ret);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%180717
 * @param   UN_IMU_DATA const * un_imudata  :lsm6dslのint型TEMP、GY、XL
 * @param   imu_float_data * imu_float　　  :float型のTEMP、GY、XL
 * @return  None
 * @brief   int型TEMP、GY、XLをfloat型に変換し、人が読みやすくする。
 * @warning
 *************************************************************************/
static void imu_int2float(UN_IMU_DATA const * un_imudata, imu_float_data * imu_float)
{
    imu_float->temp_data            = IMU_FROM_LSB_TO_degC(un_imudata->st_data.ss_temp_data );
    imu_float->acceleration_mg_x    = IMU_FROM_FS_TO_mG(un_imudata->st_data.ss_accel_data_x);
    imu_float->acceleration_mg_y    = IMU_FROM_FS_TO_mG(un_imudata->st_data.ss_accel_data_y);
    imu_float->acceleration_mg_z    = IMU_FROM_FS_TO_mG(un_imudata->st_data.ss_accel_data_z);
    imu_float->angular_rate_mrads_x = IMU_FROM_FS_TO_mRADS(un_imudata->st_data.ss_gyro_data_x);
    imu_float->angular_rate_mrads_y = IMU_FROM_FS_TO_mRADS(un_imudata->st_data.ss_gyro_data_y);
    imu_float->angular_rate_mrads_z = IMU_FROM_FS_TO_mRADS(un_imudata->st_data.ss_gyro_data_z);
}

#if 0
/**************************************************************************
 * @author  Masayuki.Fukuyama
 * @date    20220310
 * @param   imu_idx : Index of imu to send and receive
 * @param   pTxData : pointer to transmission data buffer
 * @param   pRxData : pointer to reception data buffer
 * @param   Size : amount of data to be sent
 * @return  None
 * @brief   Sends and receives a large amount of imu data in non-blocking mode using DMA.
 *          References are IMC-42605 Datasheet's 16 page
 *          [SPI TIMING CHARACTERIZATION 4-WIRE SPI MODE]
 * @warning
 * @detail
 *************************************************************************/
static void imu_SPI_TransmitReceive_DMA(uint16_t imu_idx, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(&imu_spi_info[imu_idx]);

    /* get IMU value */
    HAL_SPI_TransmitReceive_DMA(imu_spi_info[imu_idx].hspi,
                            (uint8_t *)pTxData, pRxData, Size);

}
#endif

/**************************************************************************
 * @author  Masayuki.Fukuyama
 * @date    20220315
 * @param   hspi : SPI handler  for Imu
 * @return  None
 * @brief   Turn On chip selector for SPI communication for imu
 * @warning
 * @detail
 *************************************************************************/
static void imu_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t idx_i;

    // Find SPI ch
    for (idx_i=0; idx_i<IMU_NUM_OF_MAX; idx_i++) {
        if (hspi == imu_spi_info[idx_i].hspi) {
            // Chip Selector ON 
            imu_spi_ChipSelector_On(&imu_spi_info[idx_i]);

            break;
        }
    }
}

/**************************************************************************
 * @author  Masayuki.Fukuyama/Shingo.Nishikata
 * @date    20220714
 * @param   imu_idx : Index of imu to send and receive
 * @return  Pointer to Imu's raw data storage buffer
 * @brief   receives a large amount of imu data in non-blocking mode using DMA.
 *          References are IMC-42605 Datasheet's 16 page
 *          [SPI TIMING CHARACTERIZATION 4-WIRE SPI MODE]
 * @warning
 * @detail
 *************************************************************************/
uint8_t imu_debug_flag;
static uint8_t *icm42688_GetDataInt(uint16_t imu_idx) 
{
    uint8_t *pret = NULL;

    // 転送後初回の読み出し
    if(true == dma_info[imu_idx].transmit_flag) {
        imu_debug_flag |= 0x01;
        // 受信処理待ち
        while(HAL_SPI_GetState(imu_spi_info[imu_idx].hspi) != HAL_SPI_STATE_READY){
            imu_debug_flag |= 0x02;
        };

        dma_info[imu_idx].transmit_flag = false;
        dma_info[imu_idx].data_buff_no ^= 0x01; //バッファを転送後のものに変更
    }else{
        imu_debug_flag |= 0x04;
    }

    pret = dma_info[imu_idx].data_Buff[dma_info[imu_idx].data_buff_no];

    if( (pret[9]==0) && (pret[10]==0)){
        imu_debug_flag |= 0x08;
    }else{
        imu_debug_flag |= 0x10;
    }

    return pret;
}

/*------------------------------------------------------------------------*/
/* public interfaces                                                      */
/*------------------------------------------------------------------------*/
/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   IMU 初期化通信 UI(Main側)
 * @warning
 * @detail  IMUの初期設定をする
 * @detail  上林さんのコードを改造して作成した
 *************************************************************************/
void icm42688_Initialize() {
  int i, j;
  char whoamI, rst;
  EN_IMU_ERR en_imu_err = EN_IMU_ERR_NON;

  uint16_t idx_j, idx_k;
  uint16_t setidx = 0;

  __HAL_RCC_DMA1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();

  // Spi Info.
  imu_spi_info[setidx].hspi     = &hspi1;
  imu_spi_info[setidx].hcs_port = GPIOA;
#ifdef FCX_1
  imu_spi_info[setidx].cs_pin   = GPIO_PIN_4;
#else
  imu_spi_info[setidx].cs_pin   = GPIO_PIN_15;
#endif
  // Dma Info.
  dma_info[setidx].hspi = &hspi1;
  dma_info[setidx].transmit_flag = false;
  dma_info[setidx].data_buff_no = 0;

  for (idx_j=0; idx_j<DATA_BUFF_N; idx_j++) {
      for (idx_k=0; idx_k<DMA_DATASIZE; idx_k++) {
          dma_info[setidx].data_Buff[idx_j][idx_k] = 0x00;
      }
  }

  // Set Callback
  HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_RX_COMPLETE_CB_ID, imu_TxRxCpltCallback);

  // Set CS pin to High
#ifdef FCX_1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
#endif
  // IMUを4線式SPIに設定
  IMU_SPI_Chip_Set_SPI4(&imu_spi_info[setidx]);

  // チップID問い合わせ
  whoamI = Sv_TSPI_Read_Single(&imu_spi_info[setidx], IMU_WHOAMI_ADDRESS);

  // Debug
//  printf("    [Debug] whoamI : 0x%02x .\r\n", whoamI);


  if ( whoamI != IMU_WHOAMI_RESPONSE ){
      en_imu_err = EN_IMU_ERR_WHOAMI;       /* チップIDが間違っていた場合、エラーフラグを立てる */
      s_en_imu_err[setidx] = en_imu_err;
  }

  // レジスタ値リセット
  Sv_TSPI_Write_Single(&imu_spi_info[setidx], IMU_RESET_ADDRESS, IMU_RESET_COMMAND);

  for(j=0;j<10000;j++){
      asm("nop");
      //300usほどsoftware reset. 1ms以上待つ
  }

  // レジスタ値リセット待ち
  i=0;
  while(1){
      IMU_SPI_Chip_Set_SPI4(&imu_spi_info[setidx]);  //IMUを4線式SPIに設定(RSTされているため再設定)
      rst = Sv_TSPI_Read_Single(&imu_spi_info[setidx], IMU_RESET_DONE_ADDRESS);

      // Debug
//  printf("    [Debug] rst : 0x%02x .\r\n", rst);

      if(rst==IMU_RESET_DONE_FLAG) {
          break;                                  /* リセットを確認した */
      }

      if(i>IMU_RST_WAIT) {
          en_imu_err = EN_IMU_ERR_RESET;      /* IMU_RST_WAIT回問い合わせてもリセットされないなら、エラーフラグを立てる */
          break;
      }
      i+=1;

      for(j=0;j<10000;j++){
          asm("nop");
          //300usほどsoftware reset. 1ms以上待つ
      }
  }

  /* UI(Main)側の初期設定値を送信*/
  IMU_SPI_Send_Initial_Settings(&imu_spi_info[setidx]);


  //IMU値読み出しアドレスの用意
  for(i=0; i<IMU_ITEMSIZE; i++){
      s_imu_read_transmit_data[i] = 0x00;
  }
  s_imu_read_transmit_data[0] = C_US_IMU_READ | IMU_ACC_ADDR;

  /* エラー情報を代入 */
  s_en_imu_err[setidx] = en_imu_err;
//  printf("\r\n");
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   SPI IMUリードアドレスセット関数
 *************************************************************************/
void icm42688_Int(uint16_t imu_idx) {

    // Chip Selector OFF 
    imu_spi_ChipSelector_Off(&imu_spi_info[imu_idx]);

    unsigned char l_buff_no = dma_info[imu_idx].data_buff_no ^ 0x01;

    HAL_SPI_TransmitReceive_DMA(imu_spi_info[imu_idx].hspi, 
                                (uint8_t *)s_imu_read_transmit_data, 
                                (uint8_t *)dma_info[imu_idx].data_Buff[l_buff_no],
                                DMA_DATASIZE);

    dma_info[imu_idx].transmit_flag = true; //転送開始フラグを立てる
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   SPI IMUブロッキング受信処理
 *************************************************************************/
imu_float_data icm42688_GetDataFloat_blocking(SPI_HandleTypeDef *hspi)
{
    int i;
    uint16_t l_send_data[16];
    uint16_t l_recv_data[16];

    uint16_t l_temp;

    for(i=0; i<IMU_DATA_LENGTH;i++){
        l_temp = (uint16_t)((C_US_IMU_READ | (IMU_ACC_ADDR + i)) << 8);
        l_send_data[i] = l_temp;
    }
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)l_send_data, (uint8_t *)l_recv_data, IMU_DATA_LENGTH, 10);

    UN_IMU_DATA l_imudata;
    l_imudata.st_data.ss_temp_data    = (int16_t)(((l_recv_data[0]  & 0x00ff) << 8) | (l_recv_data[1]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_x = (int16_t)(((l_recv_data[2]  & 0x00ff) << 8) | (l_recv_data[3]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_y = (int16_t)(((l_recv_data[4]  & 0x00ff) << 8) | (l_recv_data[5]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_z = (int16_t)(((l_recv_data[6]  & 0x00ff) << 8) | (l_recv_data[7]  & 0x00ff));
    l_imudata.st_data.ss_gyro_data_x  = (int16_t)(((l_recv_data[8]  & 0x00ff) << 8) | (l_recv_data[9]  & 0x00ff));
    l_imudata.st_data.ss_gyro_data_y  = (int16_t)(((l_recv_data[10] & 0x00ff) << 8) | (l_recv_data[11] & 0x00ff));
    l_imudata.st_data.ss_gyro_data_z  = (int16_t)(((l_recv_data[12] & 0x00ff) << 8) | (l_recv_data[13] & 0x00ff));

    imu_float_data l_imu;
    imu_int2float(&l_imudata, &l_imu);

    return l_imu;
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   SPI 受信処理
 *************************************************************************/
imu_float_data icm42688_GetDataFloat(uint16_t imu_idx)
{
    uint8_t *l_recv_data = NULL;
    l_recv_data = icm42688_GetDataInt(imu_idx);

    UN_IMU_DATA l_imudata;
    l_imudata.st_data.ss_temp_data    = (int16_t)(((l_recv_data[1]  & 0x00ff) << 8) | (l_recv_data[2]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_x = (int16_t)(((l_recv_data[3]  & 0x00ff) << 8) | (l_recv_data[4]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_y = (int16_t)(((l_recv_data[5]  & 0x00ff) << 8) | (l_recv_data[6]  & 0x00ff));
    l_imudata.st_data.ss_accel_data_z = (int16_t)(((l_recv_data[7]  & 0x00ff) << 8) | (l_recv_data[8]  & 0x00ff));
    l_imudata.st_data.ss_gyro_data_x  = (int16_t)(((l_recv_data[9]  & 0x00ff) << 8) | (l_recv_data[10] & 0x00ff));
    l_imudata.st_data.ss_gyro_data_y  = (int16_t)(((l_recv_data[11] & 0x00ff) << 8) | (l_recv_data[12] & 0x00ff));
    l_imudata.st_data.ss_gyro_data_z  = (int16_t)(((l_recv_data[13] & 0x00ff) << 8) | (l_recv_data[14] & 0x00ff));

    imu_float_data l_imu;
    if (EN_IMU_ERR_NON != s_en_imu_err[imu_idx]) {
        l_imu.temp_data = 0.0f;
        l_imu.angular_rate_mrads_x = 0.0f;
        l_imu.angular_rate_mrads_y = 0.0f;
        l_imu.angular_rate_mrads_z = 0.0f;
        l_imu.acceleration_mg_x    = 0.0f;
        l_imu.acceleration_mg_y    = 0.0f;
        l_imu.acceleration_mg_z    = 0.0f;
    }
    else {
        imu_int2float(&l_imudata, &l_imu);
    }

    return l_imu;
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  IMUのエラー状態
 * @brief   IMUのエラー状態を返すGet関数。0の時はエラーなし
 * @warning
 *************************************************************************/
EN_IMU_ERR icm42688_GetError(SPI_HandleTypeDef * hspi)
{
    int idx_i;
    EN_IMU_ERR ret = EN_IMU_ERR_INI;

    for (idx_i=0; idx_i<IMU_NUM_OF_MAX; idx_i++) {
        if (hspi == imu_spi_info[idx_i].hspi) {
            ret = s_en_imu_err[idx_i];
            break;
        }
    }

    return ret;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance != SPI1) return;

/** Initializes the peripherals clock
*/
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 400;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) Error_Handler();
  __HAL_RCC_SPI1_CLK_ENABLE();

  /* SPI1 DMA Init */
  /* SPI1_RX Init */
  hdma_spi1_rx.Instance = DMA1_Stream0;
  hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if(HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(hspi,hdmarx,hdma_spi1_rx);

  /* SPI1_TX Init */
  hdma_spi1_tx.Instance = DMA1_Stream1;
  hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if(HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);
}
