#ifndef __USB_COMMUNICATION_H__
#define __USB_COMMUNICATION_H__

#include <Arduino.h>
#include "battery_control.h"

// USB communication protocol
typedef struct {
    uint8_t command;
    uint8_t length;
    uint8_t data[32];
    uint8_t checksum;
} USBMessage_t;

// Command definitions
#define USB_CMD_GET_BATTERY_STATUS  0x01
#define USB_CMD_CFET_CONTROL        0x02
#define USB_CMD_PING                0x03

// Response codes
#define USB_RESP_OK                 0x80
#define USB_RESP_ERROR              0x81
#define USB_RESP_BATTERY_DATA       0x82

// Function prototypes
void usb_communication_init(void);
void usb_communication_handler(void);
void usb_send_battery_status(void);
bool usb_process_command(String& command);

#endif // __USB_COMMUNICATION_H__