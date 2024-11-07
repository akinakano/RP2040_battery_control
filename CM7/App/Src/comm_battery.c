



#include    <stdio.h>
#include    <ctype.h>
#include    <stdarg.h>
#include    "comm_battery.h"
#include    "tim.h"
#include    "gpio.h"
#include    "sci.h"

// PB8 I2C1_SCL
// PB9 I2C1_SDA

extern SMBUS_HandleTypeDef hsmbus1;
static int SMBUS_Status = 1;
static int CommandSeq;
static int ByteOffset;
struct BatteryStatus_st BatteryStatus;
uint8_t ReadBuf[32];
static uint8_t WriteBuf[4];
int Status_15V = 0;
static int Count_15V = 2;

struct BatteryCommands_st {
  uint8_t cmd;
  int   readLen;
};
static struct BatteryCommands_st BatteryCommands[] = {
  { 0x00,  }, // dummy
  { 0x24, 5 }, // Current mA
  { 0x09, 2 }, // Voltage mV
  { 0x0d, 2 }, // Capacity %
  { 0x08, 2 }, // Temperture 0.1K
  { 0x1b, 2 }, // ManufacureDate
  { 0x1c, 2 }, // SerialNumber
  { 0x00, 2 }, // ManufacturerAccess bit field
  { 0xff, 0 }, // Terminate
};

void comm_battery_Init() {

  CommandSeq = 0;
  BatteryStatus.ValidFlag = 0;

  WriteBuf[0] = 0x3f;
  WriteBuf[1] = 0x63;
  WriteBuf[2] = 0xee;
  for(int retry = 0; retry < 3; retry++) {
    HAL_StatusTypeDef ret = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, BATTERY_ADDR, WriteBuf, 3, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);
    if(ret != HAL_OK) continue;
    uint32_t state;
    while((state = HAL_SMBUS_GetState(&hsmbus1)) == HAL_SMBUS_STATE_BUSY);
    SMBUS_Status = 2;
    return;
  }
  WriteBuf[0] = 0x3e;
  WriteBuf[1] = 0x00;
  WriteBuf[2] = 0x00;
  for(int retry = 0; retry < 3; retry++) {
    HAL_StatusTypeDef ret = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, BATTERY_ADDR, WriteBuf, 3, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);
    if(ret != HAL_OK) continue;
    uint32_t state;
    while((state = HAL_SMBUS_GetState(&hsmbus1)) == HAL_SMBUS_STATE_BUSY);
    SMBUS_Status = 2;
    return;
  }
  SMBUS_Status = 0;
  return;
}

void comm_battery_tx_Handler() {

  // 1sec interval
  GPIO_HeartBeat();
  int sw = GPIO_Read_15V_SW();
  if(Status_15V != sw) {
    if(sw) {
      if(Count_15V) {
        Count_15V--;
      } else {
        GPIO_15V(Status_15V = sw);
      }
    } else {
      Count_15V = 2;
      GPIO_15V(Status_15V = sw);
    }
  } else {
    if(Count_15V) Count_15V--;
  }

  if(SMBUS_Status != 2) return;
  CommandSeq = 0;
  ByteOffset = 0;
  HAL_SMBUS_MasterRxCpltCallback(&hsmbus1);
}

void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {

  if(CommandSeq < 0) return;
  HAL_SMBUS_Master_Receive_IT(hsmbus, BATTERY_ADDR, ReadBuf + ByteOffset, BatteryCommands[CommandSeq].readLen, SMBUS_LAST_FRAME_NO_PEC);
}

void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {

  ByteOffset += BatteryCommands[CommandSeq++].readLen;

  int p = 0;
  WriteBuf[0] = BatteryCommands[CommandSeq].cmd;
  if(WriteBuf[0] != 0xff) {

    HAL_SMBUS_Master_Transmit_IT(hsmbus, BATTERY_ADDR, WriteBuf, 1, SMBUS_FIRST_FRAME);
    return;
  }

  BatteryStatus.ValidFlag |= (1 << 1);
  BatteryStatus.Current = ReadBuf[p + 1] | (ReadBuf[p + 2] << 8) | (ReadBuf[p + 3] << 16) | (ReadBuf[p + 4] << 24); p += 5;
  BatteryStatus.Voltage = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.Capacity = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.Temperture = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.ManufactureDate = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.SerialNumber = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.ManufacturerAccess = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus.ValidFlag = (1 << 1) | (1 << 0);
}

void Comm_Battery_IrqHandler() {

    if(TIM13->SR & TIM_SR_UIF){
        TIM13->SR &= ~TIM_SR_UIF;
    }
    comm_battery_tx_Handler();
}
