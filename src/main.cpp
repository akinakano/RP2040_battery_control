#include <Arduino.h>
#include <Wire.h>

#include "battery_control.h"
#include "smbus_rp2040.h"
#include "usb_communication.h"
#include "protocol_mpap.h"
#include "heartbeat_led.h"

void setup() {
    Serial.begin(115200);
    
    // Wait for USB Serial connection
    while (!Serial && millis() < 5000) {
        delay(10);
    }
    
    Serial.println("RP2040-zero Battery Control System");
    Serial.println("Initializing...");
    
    // Initialize I2C for SMBUS (GPIO26=SDA, GPIO27=SCL)
    Serial.println("Setting up I2C...");
    Wire1.setSDA(26);
    Wire1.setSCL(27);
    Wire1.begin();
    Wire1.setClock(100000);  // 100kHz for SMBUS
    Serial.println("I2C initialized on Wire1");
    
    // Initialize battery communication
    Serial.println("Initializing battery control...");
    battery_control_init();
    Serial.println("Battery control initialized");
    
    // Initialize USB communication
    Serial.println("Initializing USB communication...");
    usb_communication_init();
    Serial.println("USB communication initialized");
    
    // Initialize MPAP protocol
    Serial.println("Initializing MPAP protocol...");
    protocol_mpap_init();
    Serial.println("MPAP protocol initialized");
    
    // Initialize LED heartbeat
    Serial.println("Initializing LED heartbeat...");
    heartbeat_led_init();
    Serial.println("LED heartbeat initialized");
    
    Serial.println("Initialization complete");
    Serial.println("=== RP2040-zero Battery Control Ready ===");
    Serial.println("Type 'help' for available commands");
    Serial.println();
}

void loop() {
    // Handle battery communication (called every 100ms)
    static unsigned long last_battery_update = 0;
    static unsigned long last_debug_print = 0;
    static unsigned long last_mpap_send = 0;
    
    if (millis() - last_battery_update >= 100) {
        battery_control_handler();
        last_battery_update = millis();
    }
    
    // MPAP protocol transmission every 1000ms (1Hz)
    if (millis() - last_mpap_send >= 1000) {
        protocol_mpap_send_data();
        last_mpap_send = millis();
    }
    
    // Debug print every 30 seconds (reduced frequency)
    if (millis() - last_debug_print >= 30000) {
        Serial.println("[INFO] System running... (type 'help' for commands)");
        last_debug_print = millis();
    }
    
    // Handle USB communication
    usb_communication_handler();
    
    // Handle LED heartbeat
    heartbeat_led_handler();
    
    delay(10);
}