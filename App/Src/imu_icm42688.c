/**********************************************************************//**
* @file  imu_icm42688.c
* @brief imu icm42688と通信する関数を定義する
*
* 開発部課：AIRBG SR事業室 駆動制御課
* 作成者：高木和貴/Shingo.Nishikata(esOL)
* (C) 2022- Sony Corporation
*************************************************************************/
#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <stdbool.h>
#include "imu_icm42688.h"
#include "main.h"
#include "spi.h"

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


uint8_t dataBuffer[DMA_DATASIZE];

static EN_IMU_ERR s_en_imu_err = EN_IMU_ERR_INI;     //IMUエラーステータス
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

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @return  void
 * @brief   SPI シングル送信関数
 * @warning フレーム長=16bitが事前設定されていることを期待する。16bit以外では正常動作しない。
 *************************************************************************/
static void Sv_TSPI_Write_Single(uint8_t Reg_Addr, uint8_t Reg_value)
{
    uint8_t send_cmd[2] = {Reg_Addr, Reg_value};

    spi_Transfer(send_cmd, NULL, 2, 10);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   register address
 * @return  void
 * @brief   SPI シングル受信関数
 * @warning フレーム長=16bitが事前設定されていることを期待する。16bit以外では正常動作しない。
 * @warning 受信FIFOが空になっていることを期待する。受信FIFOはクリアされる。
 *************************************************************************/
static uint8_t Sv_TSPI_Read_Single(uint8_t Reg_Addr)
{
    uint8_t send_cmd[2];
    uint8_t recv_data[2];

    send_cmd[0] = C_US_IMU_READ | Reg_Addr;
    send_cmd[1] = 0;
    spi_Transfer((uint8_t *)&send_cmd,(uint8_t *)&recv_data, 2, 10);

//  printf("  [Debug]  spi_TransmitAndReceive Read Data : 0x%02x/0x%02x .\r\n",
//           recv_data[0], recv_data[1]);
    return recv_data[1];
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @return  None
 * @brief   IMU 初期化通信用関数
 * @warning
 *************************************************************************/
static void IMU_SPI_Send_Initial_Settings() {

    /* 初期化設定を送る*/
    spi_Transfer((uint8_t *)SCA_US_IMU_INIT_SEND_UI, NULL,
            sizeof(SCA_US_IMU_INIT_SEND_UI) / sizeof(SCA_US_IMU_INIT_SEND_UI[0][0]), 10);
}


/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%190128
 * @return  None
 * @brief   IMUのSPIをUI側もAUX側も4線式SPIに設定する
 * @detail  SW-RSTの前のコマンド発効、RST後のコマンド発効両方の前に実行が必要
 *************************************************************************/
static void IMU_SPI_Chip_Set_SPI4() {

    spi_Transfer((uint8_t *)SCA_US_IMU_SPI4_SEND_UI, NULL,
            sizeof(SCA_US_IMU_SPI4_SEND_UI), 10);

//  printf("  [Debug]  spi_Transmit ret : 0x%02x .\r\n", hal_ret);
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
void icm42688_Init() {
  int i;
  char whoamI, rst;
  EN_IMU_ERR en_imu_err = EN_IMU_ERR_NON;

  uint16_t idx_k;

  spi_Init();


  for (idx_k=0; idx_k<DMA_DATASIZE; idx_k++) {
    dataBuffer[idx_k] = 0x00;
  }
  // IMUを4線式SPIに設定
  IMU_SPI_Chip_Set_SPI4();

  // チップID問い合わせ
  whoamI = Sv_TSPI_Read_Single(IMU_WHOAMI_ADDRESS);

  // Debug
//  printf("    [Debug] whoamI : 0x%02x .\r\n", whoamI);


  if ( whoamI != IMU_WHOAMI_RESPONSE ){
      en_imu_err = EN_IMU_ERR_WHOAMI;       /* チップIDが間違っていた場合、エラーフラグを立てる */
      s_en_imu_err = en_imu_err;
  }

  // レジスタ値リセット
  Sv_TSPI_Write_Single(IMU_RESET_ADDRESS, IMU_RESET_COMMAND);
  HAL_Delay(1);

  // レジスタ値リセット待ち
  i=0;
  while(1){
      IMU_SPI_Chip_Set_SPI4();  //IMUを4線式SPIに設定(RSTされているため再設定)
      rst = Sv_TSPI_Read_Single(IMU_RESET_DONE_ADDRESS);

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

     HAL_Delay(1);
  }

  /* UI(Main)側の初期設定値を送信*/
  IMU_SPI_Send_Initial_Settings();


  //IMU値読み出しアドレスの用意
  for(i=0; i<IMU_ITEMSIZE; i++){
      s_imu_read_transmit_data[i] = 0x00;
  }
  s_imu_read_transmit_data[0] = C_US_IMU_READ | IMU_ACC_ADDR;

  /* エラー情報を代入 */
  s_en_imu_err = en_imu_err;
//  printf("\r\n");
}

void icm42688_RegisterReceiveDataCallback(void (*callback)()) {

  spi_RegisterTransferCompleteCallback(callback);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   SPI IMUリードアドレスセット関数
 *************************************************************************/
void icm42688_IrqIntervalHandler() {

  if(s_en_imu_err != EN_IMU_ERR_NON) return;
  spi_TransferDMA((uint8_t *)s_imu_read_transmit_data,
                              (uint8_t *)dataBuffer,
                              DMA_DATASIZE);
}

/**********************************************************************//**
 * @author  Kazutaka.Takaki/Shingo.Nishikata
 * @date    %%220714
 * @param   None
 * @return  None
 * @brief   SPI 受信処理
 *************************************************************************/
void icm42688_GetDataFloat(imu_float_data *l_imu) {

  if(EN_IMU_ERR_NON != s_en_imu_err) {
    l_imu->temp_data = 0.0f;
    l_imu->angular_rate_mrads_x = 0.0f;
    l_imu->angular_rate_mrads_y = 0.0f;
    l_imu->angular_rate_mrads_z = 0.0f;
    l_imu->acceleration_mg_x    = 0.0f;
    l_imu->acceleration_mg_y    = 0.0f;
    l_imu->acceleration_mg_z    = 0.0f;
  } else {
    l_imu->temp_data            = IMU_FROM_LSB_TO_degC((dataBuffer[1] << 8) | dataBuffer[2]);
    l_imu->acceleration_mg_x    = IMU_FROM_FS_TO_mG((dataBuffer[3] << 8) | dataBuffer[4]);
    l_imu->acceleration_mg_y    = IMU_FROM_FS_TO_mG((dataBuffer[5] << 8) | dataBuffer[6]);
    l_imu->acceleration_mg_z    = IMU_FROM_FS_TO_mG((dataBuffer[7] << 8) | dataBuffer[8]);
    l_imu->angular_rate_mrads_x = IMU_FROM_FS_TO_mRADS((dataBuffer[9] << 8) | dataBuffer[10]);
    l_imu->angular_rate_mrads_y = IMU_FROM_FS_TO_mRADS((dataBuffer[11] << 8) | dataBuffer[12]);
    l_imu->angular_rate_mrads_z = IMU_FROM_FS_TO_mRADS((dataBuffer[13] << 8) | dataBuffer[14]);
  }
}
