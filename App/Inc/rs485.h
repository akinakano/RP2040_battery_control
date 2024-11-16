#ifndef __RS485_H__
#define __RS485_H__

#include "stm32h747xx.h"

#define RS485_UART              (UART4)
#define RS485_BAUDRATE          (3000000)
#define RS485_IRQn              (UART4_IRQn)
#define RS485_IRQHandler        UART4_IRQHandler
#define RS485_RX_BUFF_SIZE      (4096)

void rs485_Init(void);
void rs485_send(uint8_t *buf, int len);
void RS485_IRQHandler();
void rs485_register_receive_timeout_callback(void (*func)(void));
int rs485_readByte(void);
int rs485_receivedLength(void);

#endif // __RS485_H__
