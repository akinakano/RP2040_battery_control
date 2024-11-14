
#ifndef COMM_BATTERY_H
#define COMM_BATTERY_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define COMM_BATTERY_I2C (I2C1)
#define BATTERY_ADDR (0x16) // 0x36

struct BatteryStatus_st {
  int32_t Current; // mA
  uint16_t Voltage; // mV
  uint16_t Capacity; // %
  uint16_t Temperture; // 0.1K
  uint16_t ManufactureDate; // 15:9 Year-1980, 8:5 Month, 4-0: Day
  uint16_t SerialNumber; // serial of day
  uint16_t ManufacturerAccess; // ManufacturerAccess status
  uint8_t  ValidFlag;
};

void Comm_Battery_Handler();

#endif // COMM_BATTERY_H
