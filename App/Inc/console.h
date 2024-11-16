#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include <stdint.h>

#define CONSOLE_UART       (USART2)
#define CONSOLE_BAUDRATE   (115200)
#define Console_IRQHandler USART2_IRQHandler
#define RX_BUFF_SIZE       16
#define TX_BUFF_SIZE       512

void Console_Init();
int getchar(void);
int getcharNonblock(void);

#endif // _CONSOLE_H_
