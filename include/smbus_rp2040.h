#ifndef __SMBUS_RP2040_H__
#define __SMBUS_RP2040_H__

#include <stdint.h>

#define BATTERY_ADDR 0x0B  // Detected by I2C scan (was 0x16)

// SMBUS command structure (same as STM32 version)
typedef struct {
    uint8_t cmd;
    uint8_t data_size;
} SMBusCommand_t;

// Function prototypes
bool smbus_init(void);
bool smbus_transmit(uint8_t* data, uint8_t length);
bool smbus_read_status(SMBusCommand_t* commands, uint8_t command_count, uint8_t* read_buffer);
bool smbus_read_register(uint8_t reg, uint8_t* data, uint8_t length);
bool smbus_write_register(uint8_t reg, uint8_t* data, uint8_t length);

#endif // __SMBUS_RP2040_H__