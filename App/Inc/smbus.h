#ifndef __SMBUS_H__
#define __SMBUS_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define COMM_BATTERY_I2C (I2C1)
#define BATTERY_ADDR (0x16) // 0x36

typedef struct {
  uint8_t cmd;
  uint8_t dataSize;
} SMBusCommandSt;

void SMBUS_Init();
void SMBUS_RegisterTransferCompleteCallback(void (*callback)());
int SMBUS_Transmit(uint8_t *buf, int len);
void SMBUS_ReadStatus(SMBusCommandSt *commands, int commandLength, uint8_t *buf);

#endif // __SMBUS_H__

