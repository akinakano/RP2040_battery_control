#include "battery_control.h"
#include "smbus_rp2040.h"
#include <Arduino.h>
#include <Wire.h>

// Battery command definitions (same as STM32 version)
static SMBusCommand_t battery_commands[] = {
    { 0x24, 5 }, // Current mA
    { 0x09, 2 }, // Voltage mV
    { 0x0d, 2 }, // Capacity %
    { 0x08, 2 }, // Temperature 0.1K
    { 0x1b, 2 }, // ManufactureDate
    { 0x1c, 2 }, // SerialNumber
    { 0x00, 2 }, // ManufacturerAccess bit field
};

static const uint8_t BATTERY_COMMAND_COUNT = sizeof(battery_commands) / sizeof(SMBusCommand_t);

// Global battery status
static BatteryStatus_t battery_status = {0};
static uint8_t read_buffer[32];
static bool smbus_status_ok = false;

// Private function prototypes
static bool battery_cfet_on(void);
static void parse_battery_data(void);

void battery_control_init(void) {
    Serial.println("Starting SMBUS initialization...");
    
    // Initialize SMBUS
    smbus_init();
    
    // Scan for I2C devices
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
            Serial.print("Found I2C device at address 0x");
            Serial.print(addr, HEX);
            Serial.print(" (");
            Serial.print(addr);
            Serial.println(")");
        }
    }
    Serial.println("I2C scan complete");
    Serial.print("Using battery address: 0x");
    Serial.println(BATTERY_ADDR, HEX);
    
    // Initialize battery status
    battery_status.data_valid = false;
    smbus_status_ok = false;
    
    Serial.println("Battery control initialization complete");
}

void battery_control_handler(void) {
    // First attempt CFET ON if not done yet
    if (!smbus_status_ok) {
        if (battery_cfet_on()) {
            smbus_status_ok = true;
            Serial.println("Battery CFET enabled successfully");
        } else {
            Serial.println("Failed to enable battery CFET");
            return;
        }
    }
    
    // Read battery status
    if (smbus_read_status(battery_commands, BATTERY_COMMAND_COUNT, read_buffer)) {
        parse_battery_data();
        battery_status.data_valid = true;
    } else {
        Serial.println("Failed to read battery status");
        battery_status.data_valid = false;
    }
}

bool battery_cfet_control(bool enable) {
    if (enable) {
        return battery_cfet_on();
    } else {
        // CFET OFF implementation would go here if needed
        Serial.println("CFET OFF not implemented");
        return false;
    }
}

static bool battery_cfet_on(void) {
    // First command: Write 0x63, 0xee to register 0x3f
    uint8_t cmd1_data[2] = {0x63, 0xee};
    if (!smbus_write_register(0x3f, cmd1_data, 2)) {
        Serial.println("CFET ON command 1 failed");
        return false;
    }
    
    delay(10); // Small delay between commands
    
    // Second command: Write 0x00, 0x00 to register 0x3e
    uint8_t cmd2_data[2] = {0x00, 0x00};
    if (!smbus_write_register(0x3e, cmd2_data, 2)) {
        Serial.println("CFET ON command 2 failed");
        return false;
    }
    
    return true;
}

static void parse_battery_data(void) {
    uint8_t offset = 0;
    
    // Parse current (5 bytes, signed 32-bit)
    battery_status.current_ma = (int32_t)(
        read_buffer[offset + 1] | 
        (read_buffer[offset + 2] << 8) | 
        (read_buffer[offset + 3] << 16) | 
        (read_buffer[offset + 4] << 24)
    );
    offset += 5;
    
    // Parse voltage (2 bytes, unsigned 16-bit)
    battery_status.voltage_mv = read_buffer[offset] | (read_buffer[offset + 1] << 8);
    offset += 2;
    
    // Parse capacity (2 bytes, unsigned 16-bit)
    battery_status.capacity_percent = read_buffer[offset] | (read_buffer[offset + 1] << 8);
    offset += 2;
    
    // Parse temperature (2 bytes, unsigned 16-bit)
    battery_status.temperature_01k = read_buffer[offset] | (read_buffer[offset + 1] << 8);
    offset += 2;
    
    // Parse manufacture date (2 bytes, unsigned 16-bit)
    battery_status.manufacture_date = read_buffer[offset] | (read_buffer[offset + 1] << 8);
    offset += 2;
    
    // Parse serial number (2 bytes, unsigned 16-bit)
    battery_status.serial_number = read_buffer[offset] | (read_buffer[offset + 1] << 8);
    offset += 2;
    
    // Parse manufacturer access (2 bytes, unsigned 16-bit)
    battery_status.manufacturer_access = read_buffer[offset] | (read_buffer[offset + 1] << 8);
}

void battery_print_status(void) {
    if (!battery_status.data_valid) {
        Serial.println("Battery data not valid");
        return;
    }
    
    Serial.println("=== Battery Status ===");
    Serial.print("Current: ");
    Serial.print(battery_status.current_ma);
    Serial.println(" mA");
    
    Serial.print("Voltage: ");
    Serial.print(battery_status.voltage_mv);
    Serial.println(" mV");
    
    Serial.print("Capacity: ");
    Serial.print(battery_status.capacity_percent);
    Serial.println(" %");
    
    Serial.print("Temperature: ");
    Serial.print(battery_status.temperature_01k / 10.0 - 273.15);
    Serial.println(" °C");
    
    Serial.print("Manufacture Date: 0x");
    Serial.println(battery_status.manufacture_date, HEX);
    
    Serial.print("Serial Number: ");
    Serial.println(battery_status.serial_number);
    
    Serial.print("Manufacturer Access: 0x");
    Serial.println(battery_status.manufacturer_access, HEX);
    
    Serial.println("=====================");
}

BatteryStatus_t* battery_get_status(void) {
    return &battery_status;
}