#ifndef COMM_APMP_H
#define COMM_APMP_H

#include <stdint.h>

#define COMM_APMP_OFF (0)
#define COMM_APMP_ON  (1)
#define COMM_APMP_SW  COMM_APMP_ON

#define MP_VERSION_YY  ((uint8_t)24)   //0 - 255
#define MP_VERSION_MM  ((uint8_t)10)   //0 - 255
#define MP_VERSION_DD  ((uint8_t)30)   //0 - 255
#define MP_VERSION_NN  ((uint8_t)0)   //0 - 255


//#define COMM_APMP_UART (UART8)
#define COMM_APMP_UART (UART4)

int32_t comm_apmp_read(int32_t *c);
int32_t comm_apmp_write(int32_t c);
int32_t comm_apmp_putchar(int32_t c);
int32_t comm_apmp_puts(const char *str);
int32_t comm_apmp_printf(const char *str, ... );
int32_t comm_apmp_getchar(void);
int32_t comm_apmp_getchar2(void);
char *comm_apmp_gets(char *s);

void comm_apmp_rx_handler(void);
void comm_apmp_tx_handler(void);
void send_status_mp_to_ap(void);
void send_data_on_mp_to_ap(void);

#define SEND_OFF_MP_TO_AP (0)
#define SEND_ON_MP_TO_AP  (1)

#define AP_MP_CMD_IDLE (0)
#define AP_MP_CMD_RUN  (1)

#define COMM_APMP_HEADER_H (0xab)
#define COMM_APMP_HEADER_L (0xcd)

#define COMM_APMP_SIZE (22)
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
    //uint8_t wheel_radius_right_l;
    uint16_t wheel_radius_left;
    //uint8_t wheel_radius_left_l;
    uint16_t tread;
    //uint8_t tread_l;
    uint16_t imu_gyro_scale_gain;
    //uint8_t imu_gyro_scale_gain_l;
    uint8_t rsv2_h;
    uint8_t rsv2_l;
    uint8_t rsv3_h;
    uint8_t rsv3_l;
    uint8_t crc;
}Comm_APMP_Receive;



extern Comm_APMP_Receive apmp_data;
extern uint8_t apmp_buff[COMM_APMP_SIZE];
extern uint8_t mpap_buff[COMM_MPAP_SIZE];
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

//#define COMM_MP_AP_SIZE (29)

// typedef enum{
//     MP_AP_HEAD_H = 0,
//     MP_AP_HEAD_L,
//     MP_AP_
// }

#endif