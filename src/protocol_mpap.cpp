#include "protocol_mpap.h"
#include "battery_control.h"
#include <Arduino.h>

static uint16_t time_counter = 0;

// CRC8 calculation (polynomial 0x07)
uint8_t protocol_mpap_calculate_crc(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0x00;
    
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

void protocol_mpap_init(void) {
    time_counter = 0;
    Serial.println("MPAP protocol initialized (1Hz JSON transmission)");
}

void protocol_mpap_send_data(void) {
    // Get current battery status
    BatteryStatus_t* battery_status = battery_get_status();
    
    // Create JSON string with payload_battery wrapper
    String json = "{\"payload_battery\":{";
    json += "\"count\":" + String(time_counter++) + ",";
    json += "\"id\":1,";  // マイコンID (1 or 2)
    
    if (battery_status && battery_status->data_valid) {
        json += "\"battery_current_ma\":" + String(battery_status->current_ma) + ",";
        json += "\"battery_voltage_mv\":" + String(battery_status->voltage_mv) + ",";
        json += "\"battery_capacity\":" + String(battery_status->capacity_percent) + ",";
        json += "\"battery_temperature\":" + String(battery_status->temperature_01k) + ",";
        json += "\"battery_mfg_date\":" + String(battery_status->manufacture_date) + ",";
        json += "\"battery_serial\":" + String(battery_status->serial_number) + ",";
        json += "\"battery_mfg_access\":" + String(battery_status->manufacturer_access) + ",";
        json += "\"battery_valid\":1,";
    } else {
        // No valid battery data
        json += "\"battery_current_ma\":0,";
        json += "\"battery_voltage_mv\":0,";
        json += "\"battery_capacity\":0,";
        json += "\"battery_temperature\":0,";
        json += "\"battery_mfg_date\":0,";
        json += "\"battery_serial\":0,";
        json += "\"battery_mfg_access\":0,";
        json += "\"battery_valid\":0,";
    }
    
    json += "\"system_status\":1,";
    json += "\"timestamp\":" + String(millis());
    json += "}}";  // Close payload_battery and root object
    
    // Send JSON string via Serial
    Serial.println(json);
}