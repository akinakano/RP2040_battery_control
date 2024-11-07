#ifndef COMM_MPSV_H
#define COMM_MPSV_H

#include "stm32h747xx.h"

#define SCI_COMM_UART      (USART2)
#define  MP_COMM_UART_IRQn  (USART2_IRQn)


#define MODE_CHANGE_ALL   (0x01)
#define MODE_CHANGE_TID   (0x02)
#define FORCE_CNTRL_ALL   (0x10)
#define FORCE_CNTRL_TID   (0x11)
#define POS_CNTRL_ALL     (0x12)
#define POS_CNTRL_TID     (0x13)
#define ACC_CNTRL_ALL     (0x14)
#define ACC_CNTRL_TID     (0x15)
#define VEL_CNTRL_ALL     (0x18)
#define VEL_CNTRL_TID     (0x19)
#define CFG_ORIGIN_ALL    (0xC0)
#define CFG_CUR_LIM_ALL   (0xC1)

#define TO_KEEP           (0x0)
#define TO_IDLE_MODE      (0x1)
#define TO_FORCE_MODE     (0x2)
#define TO_POS_MODE       (0x3)
#define TO_ACC_MODE       (0x4)
#define TO_VEL_MODE       (0x6)
#define TO_DIAG_MODE      (0x7)

#define MOTOR_TEMP        (0x00)
#define VM_VOLT           (0x01)
#define REF_FORCE         (0x02)
#define REF_POS           (0x03)

#define SV_STATUS_ID      (0x81)

#define MAX_SERVO_NUM     (12)
#define COMM_SV_NUM       (2)
#define SEND_STATUS_LEN   (16)

#define HEADER_SIZE       (4)
#define CRC_SIZE          (1)
#define COMMAND_SIZE_MAX  ( HEADER_SIZE + 2 * MAX_SERVO_NUM + CRC_SIZE )

typedef struct{
    uint16_t cnt;
    int16_t mr;
    int16_t hall;
    int16_t iq;
    int16_t tau;
    int16_t req;
    int16_t vdc;
    uint16_t current_state;
    int16_t crc_err;
}SV_Comm_t;

extern SV_Comm_t sv_data[COMM_SV_NUM];

typedef enum {
    NO_RECEIVING = 0,
    HEADER_RECEIVING,
    BODY_RECEIVING,
    RECEIVE_COMPLETED,
    RECEIVE_ABORTED,
    RECEIVE_FAILED,
} PACKET_RECEIVE_STATUS;

typedef enum {
    HEADER_COMMAND_ID = 0,
    HEADER_TARGET_ID,
    HEADER_PACKET_LEN,
    HEADER_REQ_STATUS,
} HEADER_MP_TO_SV;

typedef enum {
    HEADER_STATUS_ID = 0,
    HEADER_MY_SV_ID,
    HEADER_PACKET_LEN_SV,
    HEADER_REQUESTED_STATUS,
} HEADER_SV_TO_MP;

typedef enum {
    BODY_SOURCE_CURRENT = 4,
    BODY_IQ_CURRENT_L,
    BODY_IQ_CURRENT_H,
    BODY_POS_HALL_L,
    BODY_POS_HALL_H,
    BODY_POS_MR_L,
    BODY_POS_MR_H,
    BODY_PRIMARY_STATUS_L,
    BODY_PRIMARY_STATUS_H,
    BODY_REQED_STATUS_VALUE_L,
    BODY_REQED_STATUS_VALUE_H,
} BODY_SV_TO_MP;

typedef enum {
    FOOTER_CRC = 15,
} FOOTER_SV_TO_MP;

typedef enum {
    STATE_MP_TO_SV = 0,
    STATE_SV_TO_MP_UNI,
    STATE_SV_TO_MP_BROAD,
    STATE_ERROR,
} STATE_TYPE;

typedef enum {
    BROAD = 0,
    UNI,
} COMM_TARGET_TYPE;

typedef enum {
    MP_TO_SV = 0,
    SV_TO_MP,

} COMM_DIRECTION;
typedef struct {
    uint8_t* buf;
    uint32_t head;
    uint32_t tail;
    uint32_t size;
} Queue;

typedef struct {
    uint32_t framing_error;
    uint32_t parity_error;
    uint32_t overrun_error;
    uint32_t noise_error;
    uint8_t is_error;
} USART_error_count;

typedef struct {
    uint8_t  buff[COMMAND_SIZE_MAX];     // Max 5+2*N, (最大でN=6軸)
    uint8_t  command;
    uint8_t  tar_id;
    uint8_t  packet_len;
    uint8_t  req_status;
    uint8_t  param_mode[MAX_SERVO_NUM];
    int16_t  force[MAX_SERVO_NUM];
    int16_t  vel[MAX_SERVO_NUM];
    int16_t  pos[MAX_SERVO_NUM];
    uint8_t  crc;
    uint8_t  crc_err;
    uint8_t send_buff_remained;
    uint8_t  receive_buff_remained;
    uint8_t  to_diag_mode;
    PACKET_RECEIVE_STATUS  receive_status;
    uint8_t  received_byte_num;
    int32_t  tar_pos_decoded;     // 通信で指示されたpos値をSV内のpos値にデコードしたもの
    int32_t  tar_force_decoded;   // 通信で指示されたForce値をSV内のIq値にデコードしたもの
} COM_MP_TO_SV;

typedef struct {
    uint8_t buff[SEND_STATUS_LEN];
    uint8_t status_id;
    uint8_t my_sv_id;
    uint8_t packet_len;
    uint8_t req_status;
    uint8_t source_current;
    uint16_t Iq_current;
    uint16_t position_hall;
    uint16_t position_mr;
    uint16_t primary_status;
    uint16_t req_status_value;
    uint8_t crc;
    uint8_t send_buff_remained;
    uint8_t ack_required;
    PACKET_RECEIVE_STATUS  receive_status;
} COM_SV_TO_MP;

typedef struct {
    uint8_t  buff[COMMAND_SIZE_MAX];     // Max 5+2*N, (最大でN=6軸)
    uint8_t  direction;

    // MP_TO_SV 受信時パラメータ
    uint8_t  command;
    uint8_t  tar_id;
    uint8_t  dest;
    uint8_t  packet_len;
    uint8_t  req_status;
    uint8_t  param_mode[MAX_SERVO_NUM];
    int16_t  force[MAX_SERVO_NUM];
    int16_t  pos[MAX_SERVO_NUM];

    // SV_TO_MP 受信時パラメータ
    uint8_t  status_id;
    uint8_t  sending_sv_id;
    // uint8_t packet_len;
    uint8_t  reqed_status;
    int16_t  source_current;
    int16_t  iq_current;
    int16_t  pos_hall;
    int16_t  pos_mr;
    int16_t  primary_status;
    int16_t  reqed_status_value;

    uint8_t  crc;
    uint8_t  crc_err;
    uint8_t  receive_buff_remained;
    PACKET_RECEIVE_STATUS  receive_status;
    uint8_t  received_byte_num;
} COM_STATUS;

typedef struct {
    uint8_t status;
    uint8_t err_info_dir;
    uint8_t err_info_status;
    uint16_t err_info_flag;
    uint16_t err_info_temp;
    uint16_t err_info_cnt;
} STATE_MACHINE;

void comm_parser_Init( void );
COM_STATUS * get_com_status_handle( void );
STATE_MACHINE * get_com_state_machine();
COM_MP_TO_SV * get_send_cmd_handle();
COM_MP_TO_SV * get_send_mode_handle();
void send_cmd_broadcast_data(COM_MP_TO_SV * mp_sv, uint8_t cmd, uint8_t sv_num, int16_t param[], uint8_t req_status);
void send_mode_broadcast_data(COM_MP_TO_SV * mp_sv, uint8_t sv_num, uint8_t mode[], uint8_t req_status);

#endif /* INC_COMM_IRQ_HANDLER_H_ */
