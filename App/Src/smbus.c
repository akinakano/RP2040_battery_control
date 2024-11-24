#include "smbus.h"
#include "main.h"

SMBUS_HandleTypeDef hsmbus1;
static uint8_t *ReadBuf;
static int ReadPoint;
static int CommandNo;
static SMBusCommandSt *Commands;
static int CommandLength;
static uint8_t WriteBuf[4];

static void (*transferCompleteCallback)(void) = (void (*)())NULL;
static void SendCommand(SMBusCommandSt *command);

void SMBUS_Init() {

  RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;

  hsmbus1.Instance = I2C1;
  hsmbus1.Init.Timing = 0x307075B1;
  hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus1.Init.OwnAddress1 = 2;
  hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus1.Init.OwnAddress2 = 0;
  hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
  hsmbus1.Init.SMBusTimeout = 0x000085B8;
  if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK) Error_Handler();
  if (HAL_SMBUS_ConfigDigitalFilter(&hsmbus1, 0) != HAL_OK) Error_Handler();
}

void SMBUS_RegisterTransferCompleteCallback(void (*callback)()) {

  transferCompleteCallback = callback;
}

int SMBUS_Transmit(uint8_t *buf, int len) {

  CommandNo = -1;
  for(int retry = 0; retry < 3; retry++) {
    HAL_StatusTypeDef ret = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, BATTERY_ADDR, buf, len, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);
    if(ret != HAL_OK) continue;
    uint32_t state;
    while((state = HAL_SMBUS_GetState(&hsmbus1)) == HAL_SMBUS_STATE_BUSY);
    return 0;
  }
  return -1;
}

void SMBUS_ReadStatus(SMBusCommandSt *commands, int len, uint8_t *buf) {

  ReadBuf = buf;
  ReadPoint = 0;
  Commands = commands;
  CommandNo = 0;
  CommandLength = len;

  SendCommand(&commands[CommandNo]);
}

static void SendCommand(SMBusCommandSt *command) {

  WriteBuf[0] = command->cmd;
  HAL_SMBUS_Master_Transmit_IT(&hsmbus1, BATTERY_ADDR, WriteBuf, 1, SMBUS_FIRST_FRAME);
}

void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {

  if(CommandNo < 0) return;
  HAL_SMBUS_Master_Receive_IT(hsmbus, BATTERY_ADDR, ReadBuf + ReadPoint, Commands[CommandNo].dataSize, SMBUS_LAST_FRAME_NO_PEC);
}

void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {

  ReadPoint += Commands[CommandNo++].dataSize;
  if(CommandNo < CommandLength) {
    SendCommand(&Commands[CommandNo]);
    return;
  }
  CommandNo = -1;
  if(transferCompleteCallback) transferCompleteCallback();
}

void I2C1_EV_IRQHandler(void) {

  HAL_SMBUS_EV_IRQHandler(&hsmbus1);
}

void I2C1_ER_IRQHandler(void) {

  HAL_SMBUS_ER_IRQHandler(&hsmbus1);
}
