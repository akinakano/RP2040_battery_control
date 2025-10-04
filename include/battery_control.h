#ifndef __BATTERY_CONTROL_H__
#define __BATTERY_CONTROL_H__

#include <stdint.h>

// Battery status structure (same as STM32 version)
typedef struct {
    int32_t current_ma;           // Current in mA
    uint16_t voltage_mv;          // Voltage in mV
    uint16_t capacity_percent;    // Capacity in %
    uint16_t temperature_01k;     // Temperature in 0.1K
    uint16_t manufacture_date;    // 15:9 Year-1980, 8:5 Month, 4-0: Day
    uint16_t serial_number;       // Serial number of day
    uint16_t manufacturer_access; // ManufacturerAccess status
    bool data_valid;              // Data validity flag
} BatteryStatus_t;

// Function prototypes
void battery_control_init(void);
void battery_control_handler(void);
bool battery_cfet_control(bool enable);
void battery_print_status(void);
BatteryStatus_t* battery_get_status(void);

#endif // __BATTERY_CONTROL_H__