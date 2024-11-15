#include    <stdio.h>
#include    <ctype.h>
#include    <stdarg.h>
#include    "comm_battery.h"
#include    "main.h"

extern SMBUS_HandleTypeDef hsmbus1;
static int SMBUS_Status = 1;
static int CommandSeq;
static int ByteOffset;
struct BatteryStatus_st BatteryStatus;
uint8_t ReadBuf[32];
static uint8_t WriteBuf[4];

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

void comm_battery_init(void) {

  RCC->D2CCIP2R = ((uint32_t)(RCC->D2CCIP2R) & ~RCC_D2CCIP2R_I2C123SEL) | RCC_I2C123CLKSOURCE_D2PCLK1;

  // PB6     ------> I2C1_SCL
  // PB7     ------> I2C1_SDA
  GPIOB->MODER = ((uint32_t)(GPIOB->MODER) & ~GPIO_MODER_MODE6_Msk) | (2 << GPIO_MODER_MODE6_Pos);
  GPIOB->OTYPER = ((uint32_t)(GPIOB->OTYPER) & ~GPIO_OTYPER_OT6_Msk) | (1 << GPIO_OTYPER_OT6_Pos);
  GPIOB->PUPDR = ((uint32_t)(GPIOB->PUPDR) & ~GPIO_PUPDR_PUPD6_Msk) | (1 << GPIO_PUPDR_PUPD6_Pos);
  GPIOB->AFR[0] = ( ( GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 4 << GPIO_AFRL_AFSEL6_Pos));
  GPIOB->MODER = ((uint32_t)(GPIOB->MODER) & ~GPIO_MODER_MODE7_Msk) | (2 << GPIO_MODER_MODE7_Pos);
  GPIOB->OTYPER = ((uint32_t)(GPIOB->OTYPER) & ~GPIO_OTYPER_OT7_Msk) | (1 << GPIO_OTYPER_OT7_Pos);
  GPIOB->PUPDR = ((uint32_t)(GPIOB->PUPDR) & ~GPIO_PUPDR_PUPD7_Msk) | (1 << GPIO_PUPDR_PUPD7_Pos);
  GPIOB->AFR[0] = ( ( GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL7_Msk ) | ( 4 << GPIO_AFRL_AFSEL7_Pos));

  RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;
  uint32_t dummy = RCC->APB1LENR;
  UNUSED(dummy);

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

static void Battery_CFET_ON() {

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

void Comm_Battery_Handler() {

  if(SMBUS_Status == 1) Battery_CFET_ON();
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
