#include "usb_communication.h"
#include <Arduino.h>

void usb_communication_init(void) {
    Serial.begin(115200);
    Serial.println("USB Communication initialized");
}

void usb_communication_handler(void) {
    // Handle incoming USB commands
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() > 0) {
            Serial.print("> "); // Show command prompt
            Serial.println(command);
            usb_process_command(command);
        }
    }
}

bool usb_process_command(String& command) {
    if (command == "ping") {
        Serial.println("pong");
        return true;
    }
    else if (command == "status") {
        usb_send_battery_status();
        return true;
    }
    else if (command == "cfet_on") {
        if (battery_cfet_control(true)) {
            Serial.println("CFET enabled");
        } else {
            Serial.println("CFET enable failed");
        }
        return true;
    }
    else if (command == "cfet_off") {
        if (battery_cfet_control(false)) {
            Serial.println("CFET disabled");
        } else {
            Serial.println("CFET disable failed");
        }
        return true;
    }
    else if (command == "help") {
        Serial.println("=== Available Commands ===");
        Serial.println("  ping     - Test communication");
        Serial.println("  status   - Get battery status (JSON format)");
        Serial.println("  cfet_on  - Enable battery CFET");
        Serial.println("  cfet_off - Disable battery CFET");
        Serial.println("  help     - Show this help");
        Serial.println("========================");
        return true;
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(command);
        Serial.println("Type 'help' for available commands");
        return false;
    }
}

void usb_send_battery_status(void) {
    BatteryStatus_t* status = battery_get_status();
    
    if (!status->data_valid) {
        Serial.println("ERROR: Battery data not valid");
        return;
    }
    
    // Send battery status in JSON format for easy parsing by AP
    Serial.println("{");
    Serial.print("  \"current_ma\": ");
    Serial.print(status->current_ma);
    Serial.println(",");
    
    Serial.print("  \"voltage_mv\": ");
    Serial.print(status->voltage_mv);
    Serial.println(",");
    
    Serial.print("  \"capacity_percent\": ");
    Serial.print(status->capacity_percent);
    Serial.println(",");
    
    Serial.print("  \"temperature_celsius\": ");
    Serial.print(status->temperature_01k / 10.0 - 273.15);
    Serial.println(",");
    
    Serial.print("  \"manufacture_date\": ");
    Serial.print(status->manufacture_date);
    Serial.println(",");
    
    Serial.print("  \"serial_number\": ");
    Serial.print(status->serial_number);
    Serial.println(",");
    
    Serial.print("  \"manufacturer_access\": ");
    Serial.print(status->manufacturer_access);
    Serial.println(",");
    
    Serial.print("  \"data_valid\": ");
    Serial.print(status->data_valid ? "true" : "false");
    Serial.println();
    
    Serial.println("}");
}