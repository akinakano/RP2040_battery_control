#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#define GPIO_DEBUG_LED_1  (1 << 0)
#define GPIO_DEBUG_LED_2  (1 << 1)
#define GPIO_DEBUG_LED_3  (1 << 2)
#define GPIO_DEBUG_LED_4  (1 << 3)

#define PA0_ON()                    do { GPIOA->BSRR = 0x00000001; } while(0)
#define PA0_OFF()                   do { GPIOA->BSRR = 0x00010000; } while(0)
#define PA1_ON()                    do { GPIOA->BSRR = 0x00000002; } while(0)
#define PA1_OFF()                   do { GPIOA->BSRR = 0x00020000; } while(0)

extern int BatterySBUS_Supported;
extern int Power15V;

void GPIO_Init(void);
void GPIO_DebugLedOn(uint16_t idx);
void GPIO_DebugLedOff(uint16_t idx);
void PowerControl_Handler();
void Heartbeat_Handler(void);

#endif
