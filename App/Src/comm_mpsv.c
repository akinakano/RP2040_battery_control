#include <stm32h747xx.h>
#include "comm_mpsv.h"
#include "crc8.h"
#include "rs485.h"

SV_Comm_t SvData[2][COMM_SV_NUM];
int SvDataBank = 0;

static uint8_t txBuf[32];
static uint8_t rxBuf[32];
static void receive_timeout_callback(void);

void comm_mpsv_Init( void ) {

  rs485_Init();
  rs485_register_receive_timeout_callback(&receive_timeout_callback);

}

static void ReceiveCommandParse(uint8_t *buf, int len, int crcState) {

  int nextBank = SvDataBank ^ 1;
  int svId = buf[MPSVPacket_TargetID] - 1;
  SV_Comm_t *data = &SvData[nextBank][svId];
  data->cnt++;
  data->mr = (buf[MPSVPacket_PosMrH] << 8) | buf[MPSVPacket_PosMrL];
  data->hall = (buf[MPSVPacket_PosHallH] << 8) | buf[MPSVPacket_PosHallL];
  data->iq = (buf[MPSVPacket_IqCurrentH] << 8) | buf[MPSVPacket_IqCurrentL];
  data->req = (buf[MPSVPacket_ReqedStatusValH] << 8) | buf[MPSVPacket_ReqedStatusValL];
  data->vdc = buf[MPSVPacket_SourceCurrent];
  data->current_state = buf[MPSVPacket_PrimaryStatusL] & 3;
  data->crc_err = crcState;
}

static void ReceiveDataParse(void) {

  int packetLen = sizeof(rxBuf) - 1;
  int rxP = 0;
  int dn = 0;
  int packetLength = rs485_receivedLength();
  for(int i = 0; i < packetLength; i++) {
    if(rxP >= sizeof(rxBuf)) break;
    int d = rs485_readByte();
    if(d == SV_STATUS_ID) {
      rxP = 0;
    }
    rxBuf[rxP++] = d;
    if(rxP == 2 + 1) packetLen = d;
    if(rxP == packetLen) {
      uint8_t crc = calc_crc(rxBuf, packetLen - 1);
      ReceiveCommandParse(rxBuf, packetLen, d == crc ? 0 : 1);
      rxP = 0;
      dn++;
    }
  }
  SvDataBank ^= 1;
}

static void receive_timeout_callback(void) {

  ReceiveDataParse();
}

void pack_2Byte_data(uint8_t * buff, int16_t _data) {

  if(_data < 0) _data = 65536 + _data;
  buff[1] = _data % 256;
  buff[0] = (_data - buff[1]) >> 8;
}

void send_mode_broadcast_data(uint8_t sv_num, uint8_t mode[], uint8_t req_status) {

    txBuf[MPSVPacket_CommandID] = 0x01;
    txBuf[MPSVPacket_TargetID] = 0x00;
    txBuf[MPSVPacket_PacketLen] = 5 + sv_num;
    txBuf[MPSVPacket_ReqStatus] = req_status;
    for(int i = 0; i < sv_num; i++) {
      txBuf[MPSVPacket_ReqStatus + 1 + i] = mode[i];
    }
    txBuf[MPSVPacket_ReqStatus + sv_num + 1] = calc_crc(txBuf, 4 + sv_num + 1 - 1);
    rs485_send(txBuf, 5 + sv_num);
}

void send_cmd_broadcast_data(uint8_t cmd, uint8_t sv_num, int16_t param[], uint8_t req_status) {

  txBuf[MPSVPacket_CommandID] = cmd;
  txBuf[MPSVPacket_TargetID] = 0x00;
  txBuf[MPSVPacket_PacketLen] = 5 + sv_num * 2;
  txBuf[MPSVPacket_ReqStatus] = req_status;
  for(int i = 0; i < sv_num; i++) {
    if(cmd == FORCE_CNTRL_ALL) {
      pack_2Byte_data(&txBuf[MPSVPacket_ReqStatus + 1 + 2 * i], param[i]);
    } else if(cmd == POS_CNTRL_ALL) {
      pack_2Byte_data(&txBuf[MPSVPacket_ReqStatus + 1 + 2 * i], param[i]);
    } else if(cmd == VEL_CNTRL_ALL) {
      pack_2Byte_data(&txBuf[MPSVPacket_ReqStatus + 1 + 2 * i], param[i]);
    } else if(cmd == ACC_CNTRL_ALL) {
      pack_2Byte_data(&txBuf[MPSVPacket_ReqStatus + 1 + 2 * i], param[i]);
    }
  }
  txBuf[MPSVPacket_ReqStatus + sv_num * 2 + 1] = calc_crc(txBuf, 4 + sv_num * 2 + 1 - 1);
  rs485_send(txBuf, 5 + sv_num * 2);
}
