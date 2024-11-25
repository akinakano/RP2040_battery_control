#include <stdio.h>
#include "comm_battery.h"
#include "smbus.h"

static int SMBUS_Status = 1;
BatteryStatusSt BatteryStatus[2];
int BatteryStatusBank = 0;

uint8_t ReadBuf[32];
static uint8_t WriteBuf[4];

static SMBusCommandSt BatteryCommands[] = {
  { 0x24, 5 }, // Current mA
  { 0x09, 2 }, // Voltage mV
  { 0x0d, 2 }, // Capacity %
  { 0x08, 2 }, // Temperture 0.1K
  { 0x1b, 2 }, // ManufacureDate
  { 0x1c, 2 }, // SerialNumber
  { 0x00, 2 }, // ManufacturerAccess bit field
};

static void comm_batteryTransferCallback();

void comm_battery_init(void) {

  SMBUS_Init();
  SMBUS_RegisterTransferCompleteCallback(&comm_batteryTransferCallback);
}

static int Battery_CFET_ON() {


  WriteBuf[0] = 0x3f;
  WriteBuf[1] = 0x63;
  WriteBuf[2] = 0xee;
  int err = SMBUS_Transmit(WriteBuf, 3);

  WriteBuf[0] = 0x3e;
  WriteBuf[1] = 0x00;
  WriteBuf[2] = 0x00;
  err |= SMBUS_Transmit(WriteBuf, 3);
  return err;
}

void Comm_Battery_Handler() {

  if(SMBUS_Status == 1) {
    if(!Battery_CFET_ON()) SMBUS_Status = 2;
  }
  if(SMBUS_Status != 2) return;

  SMBUS_ReadStatus(BatteryCommands, sizeof(BatteryCommands) / sizeof(SMBusCommandSt), ReadBuf);
}

static void comm_batteryTransferCallback() {

  int nextBank = BatteryStatusBank ^ 1;
  int p = 0;
  BatteryStatus[nextBank].Current = ReadBuf[p + 1] | (ReadBuf[p + 2] << 8) | (ReadBuf[p + 3] << 16) | (ReadBuf[p + 4] << 24); p += 5;
  BatteryStatus[nextBank].Voltage = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus[nextBank].Capacity = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus[nextBank].Temperture = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus[nextBank].ManufactureDate = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus[nextBank].SerialNumber = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatus[nextBank].ManufacturerAccess = ReadBuf[p] | (ReadBuf[p + 1] << 8); p += 2;
  BatteryStatusBank = nextBank;
}
