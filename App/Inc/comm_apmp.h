#ifndef COMM_APMP_H
#define COMM_APMP_H

#include <stdint.h>

#define MP_VERSION_YY  ((uint8_t)24)   //0 - 255
#define MP_VERSION_MM  ((uint8_t)10)   //0 - 255
#define MP_VERSION_DD  ((uint8_t)30)   //0 - 255
#define MP_VERSION_NN  ((uint8_t)0)   //0 - 255

void Comm_APMP_Rx_Handler(uint8_t* buf, uint32_t len);
void Comm_APMP_ErrorCheckInterval();
void Comm_APMP_Parse_IrqHandler(void);
void Comm_APMP_Tx_Handler(void);
void send_data_on_mp_to_ap(void);

#define SEND_OFF_MP_TO_AP (0)
#define SEND_ON_MP_TO_AP  (1)

#define APMP_RX_BUFF_SIZE 256
#define MPAP_TX_BUFF_SIZE 256

#define AP_MP_CMD_IDLE (0)
#define AP_MP_CMD_RUN  (1)

#define COMM_APMP_HEADER_H (0xab)
#define COMM_APMP_HEADER_L (0xcd)

#define COMM_MPAP_SIZE (93)

#define COMM_DIR_X (1)
#define COMM_DIR_Y (-1)
#define COMM_DIR_Z (1)
#define VXY_MPS_TO_INT16t  (5461.167f)
#define WZ_RADPS_TO_INT16t ((float)32768.0/SMC_PI_F/2.0)
#define VXY_INT16t_TO_MPS  (0.000183111f)
#define WZ_INT16t_TO_RADPS ((float)SMC_PI_F/32768.0)    //指令値の最大値は20240822時点で±π
#define PXY_M_TO_INT16t  (10000.0f)
#define QZ_RAD_TO_INT16t ((float)32768.0/SMC_PI_F)
#define PXY_INT16t_TO_M  (0.0001f)
#define QZ_INT16t_TO_RAD ((float)SMC_PI_F/32768.0)
#define GYRO_SCALE_GAIN_INT16t_TO_FLOAT ((float)(0.1/32768.0))

#define APMP_COMM_LOST_EMERGENCY
#define APMP_COMM_LOST_COUNT     (5) //0.6s

typedef struct{
    uint8_t head_h;
    uint8_t head_l;
    uint8_t cmd;
    uint8_t vx_cmd_h;
    uint8_t vx_cmd_l;
    uint8_t vy_cmd_h;
    uint8_t vy_cmd_l;
    uint8_t wz_cmd_h;
    uint8_t wz_cmd_l;
    uint16_t wheel_radius_right;
    uint16_t wheel_radius_left;
    uint16_t tread;
    uint16_t imu_gyro_scale_gain;
    uint8_t rsv2_h;
    uint8_t rsv2_l;
    uint8_t rsv3_h;
    uint8_t rsv3_l;
    uint8_t crc;
}Comm_APMP_Receive;

extern Comm_APMP_Receive apmp_data;
extern uint16_t dbg_flag_mpap;

#define STATE_MP_TO_AP_NORMAL           (0x00)
#define STATE_MP_TO_AP_ERROR_SVCOMM     (0x01)
#define STATE_MP_TO_AP_ERROR_APCOMM     (0x02)  //0x04以降は姿勢制御のデバッグに用いる
#define FLAG_BVC_LIM_CENTORIPETAL_FORCE (0x04)
#define FLAG_BVC_LIM_ACC                (0x08)    
#define FLAG_BVC_REST_DETECT            (0x10)  //APからもらったトレッドまたはホイールの値を更新できていない
#define FLAG_BVC_LIM_MOT_TORQUE_R       (0x20)
#define FLAG_BVC_LIM_MOT_TORQUE_L       (0x40)
#define FLAG_BVC_IMU_CALIB_ERR          (0x80)

extern uint16_t state_mp_to_ap;

#endif
