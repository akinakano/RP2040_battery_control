#ifndef __PROTOCOL_MPAP_H__
#define __PROTOCOL_MPAP_H__

#include <stdint.h>

// MPAP protocol packet structure (25 bytes total)
typedef struct {
    uint8_t header;               // Header byte (0xAA)
    uint16_t time_count;          // Time counter
    int32_t battery_current_ma;   // Battery current in mA
    uint16_t battery_voltage_mv;  // Battery voltage in mV
    uint8_t battery_capacity;     // Battery capacity percentage (0-100)
    uint16_t battery_temperature; // Battery temperature
    uint16_t battery_mfg_date;    // Manufacturing date
    uint16_t battery_serial;      // Serial number
    uint16_t battery_mfg_access;  // Manufacturing access
    uint8_t battery_valid;        // Battery data valid flag (0/1)
    uint8_t system_status;        // System status
    uint8_t reserved;             // Reserved byte
    uint8_t crc;                  // CRC8 checksum
} __attribute__((packed)) ProtocolMPAP_t;

// Protocol constants
#define MPAP_HEADER_BYTE    0xAA
#define MPAP_PACKET_SIZE    25
#define MPAP_SEND_RATE_HZ   50    // 50Hz transmission rate

// Function prototypes
void protocol_mpap_init(void);
void protocol_mpap_send_data(void);
uint8_t protocol_mpap_calculate_crc(const uint8_t* data, uint8_t length);

#endif // __PROTOCOL_MPAP_H__