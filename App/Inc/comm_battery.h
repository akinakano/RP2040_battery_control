
#ifndef __COMM_BATTERY_H__
#define __COMM_BATTERY_H__

#include <stdint.h>

typedef struct {
  int32_t Current; // mA
  uint16_t Voltage; // mV
  uint16_t Capacity; // %
  uint16_t Temperture; // 0.1K
  uint16_t ManufactureDate; // 15:9 Year-1980, 8:5 Month, 4-0: Day
  uint16_t SerialNumber; // serial of day
  uint16_t ManufacturerAccess; // ManufacturerAccess status
} BatteryStatusSt;

extern BatteryStatusSt BatteryStatus[];
extern int BatteryStatusBank;

void comm_battery_init(void);
void Comm_Battery_Handler();

#endif // __COMM_BATTERY_H__
