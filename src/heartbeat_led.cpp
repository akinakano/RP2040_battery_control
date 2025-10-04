#include "heartbeat_led.h"
#include "battery_control.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// WS2812 LED strip object
static Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// LED state variables
static bool led_state = false;
static uint32_t current_color = LED_COLOR_BLUE;
static unsigned long last_heartbeat = 0;

void heartbeat_led_init(void) {
    // Initialize NeoPixel strip
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.clear();
    strip.show();
    
    // Show initialization pattern (blue blink 3 times)
    for (int i = 0; i < 3; i++) {
        strip.setPixelColor(0, LED_COLOR_BLUE);
        strip.show();
        delay(200);
        strip.clear();
        strip.show();
        delay(200);
    }
    
    Serial.println("Heartbeat LED initialized (GPIO16, WS2812)");
}

void heartbeat_led_handler(void) {
    // Get current battery status
    BatteryStatus_t* battery_status = battery_get_status();
    
    // Update LED color based on battery state
    if (battery_status && battery_status->data_valid) {
        if (battery_status->capacity_percent > 50) {
            current_color = LED_COLOR_GREEN;    // Good battery level - green
        } else if (battery_status->capacity_percent > 20) {
            current_color = LED_COLOR_YELLOW;   // Warning level
        } else {
            current_color = LED_COLOR_RED;      // Critical level
        }
    } else {
        current_color = LED_COLOR_WHITE;          // No battery data - error
    }
    
    // Heartbeat logic - toggle every 1 second
    if (millis() - last_heartbeat >= 1000) {
        led_state = !led_state;
        
        if (led_state) {
            // LED ON - show current color
            strip.setPixelColor(0, current_color);
        } else {
            // LED OFF
            strip.clear();
        }
        
        strip.show();
        last_heartbeat = millis();
    }
}