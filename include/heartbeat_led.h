#ifndef __HEARTBEAT_LED_H__
#define __HEARTBEAT_LED_H__

#include <stdint.h>

// WS2812 LED configuration
#define LED_PIN 16           // GPIO16 for WS2812 LED
#define LED_COUNT 1          // Single LED
#define LED_BRIGHTNESS 50    // LED brightness (0-255)

// LED colors for different states
#define LED_COLOR_OFF     0x000000  // Off
#define LED_COLOR_WHITE   0xFFFFFF  // Error (white)
#define LED_COLOR_GREEN   0x00FF00  // Full battery (green)
#define LED_COLOR_YELLOW  0xFFFF00  // Warning/low battery (yellow)
#define LED_COLOR_RED     0xFF0000  // critical (red)　RP2040-zeroボードの購入先によってLEDの種類が異なるため、緑と赤を入れ替える必要あり
#define LED_COLOR_BLUE    0x0000FF  // Initialization (blue)

// Function prototypes
void heartbeat_led_init(void);
void heartbeat_led_handler(void);

#endif // __HEARTBEAT_LED_H__