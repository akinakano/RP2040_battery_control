#ifndef COMM_MPSV_H
#define COMM_MPSV_H

#include "stm32h747xx.h"

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

#define COMM_SV_NUM       (2)

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
} SV_Comm_t;

extern SV_Comm_t SvData[][COMM_SV_NUM];
extern int SvDataBank;

typedef enum {
  MPSVPacket_CommandID = 0,
  MPSVPacket_TargetID,
  MPSVPacket_PacketLen,
  MPSVPacket_ReqStatus,
  MPSVPacket_SourceCurrent,
  MPSVPacket_IqCurrentH,
  MPSVPacket_IqCurrentL,
  MPSVPacket_PosHallH,
  MPSVPacket_PosHallL,
  MPSVPacket_PosMrH,
  MPSVPacket_PosMrL,
  MPSVPacket_PrimaryStatusH,
  MPSVPacket_PrimaryStatusL,
  MPSVPacket_ReqedStatusValH,
  MPSVPacket_ReqedStatusValL,
  MPSVPacket_CRC,
} ReceiveState;

void comm_mpsv_Init( void );
void send_cmd_broadcast_data(uint8_t cmd, uint8_t sv_num, int16_t param[], uint8_t req_status);
void send_mode_broadcast_data(uint8_t sv_num, uint8_t mode[], uint8_t req_status);

#endif /* INC_COMM_IRQ_HANDLER_H_ */
