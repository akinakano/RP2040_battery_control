#include "smbus_rp2040.h"
#include <Wire.h>
#include <Arduino.h>

// Use Wire1 for GPIO26/27

static bool smbus_initialized = false;

bool smbus_init(void) {
    if (smbus_initialized) {
        return true;
    }
    
    // I2C is already initialized in main.cpp with Wire1.begin()
    // Clock is already set to 100kHz in main.cpp
    
    smbus_initialized = true;
    Serial.println("SMBUS initialized (I2C @ 100kHz)");
    return true;
}

bool smbus_transmit(uint8_t* data, uint8_t length) {
    if (!smbus_initialized) {
        Serial.println("SMBUS not initialized");
        return false;
    }
    
    Wire1.beginTransmission(BATTERY_ADDR);
    
    for (uint8_t i = 0; i < length; i++) {
        Wire1.write(data[i]);
    }
    
    uint8_t result = Wire1.endTransmission();
    
    if (result != 0) {
        Serial.print("SMBUS transmit error: ");
        Serial.println(result);
        return false;
    }
    
    return true;
}

bool smbus_read_register(uint8_t reg, uint8_t* data, uint8_t length) {
    if (!smbus_initialized) {
        Serial.println("SMBUS not initialized");
        return false;
    }
    
    // Send register address
    Wire1.beginTransmission(BATTERY_ADDR);
    Wire1.write(reg);
    uint8_t result = Wire1.endTransmission(false); // Keep connection open
    
    if (result != 0) {
        Serial.print("SMBUS write register error: ");
        Serial.println(result);
        return false;
    }
    
    // Read data
    uint8_t bytes_received = Wire1.requestFrom(BATTERY_ADDR, length);
    
    if (bytes_received != length) {
        Serial.print("SMBUS read error: expected ");
        Serial.print(length);
        Serial.print(" bytes, got ");
        Serial.println(bytes_received);
        return false;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire1.read();
    }
    
    return true;
}

bool smbus_write_register(uint8_t reg, uint8_t* data, uint8_t length) {
    if (!smbus_initialized) {
        Serial.println("SMBUS not initialized");
        return false;
    }
    
    Wire1.beginTransmission(BATTERY_ADDR);
    Wire1.write(reg);
    
    for (uint8_t i = 0; i < length; i++) {
        Wire1.write(data[i]);
    }
    
    uint8_t result = Wire1.endTransmission();
    
    if (result != 0) {
        Serial.print("SMBUS write register error: ");
        Serial.println(result);
        return false;
    }
    
    return true;
}

bool smbus_read_status(SMBusCommand_t* commands, uint8_t command_count, uint8_t* read_buffer) {
    if (!smbus_initialized) {
        Serial.println("SMBUS not initialized");
        return false;
    }
    
    uint8_t buffer_offset = 0;
    
    for (uint8_t i = 0; i < command_count; i++) {
        bool success = smbus_read_register(commands[i].cmd, 
                                         &read_buffer[buffer_offset], 
                                         commands[i].data_size);
        
        if (!success) {
            Serial.print("Failed to read command 0x");
            Serial.print(commands[i].cmd, HEX);
            Serial.println();
            return false;
        }
        
        buffer_offset += commands[i].data_size;
        
        // Small delay between commands
        delay(1);
    }
    
    return true;
}