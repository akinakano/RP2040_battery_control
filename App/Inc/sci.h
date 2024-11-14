#ifndef _SCI_H_
#define _SCI_H_

#include <stdint.h>

#define SCI_UART     (USART2)
#define APB_CLK      (120000000)
#define SCI_BAUDRATE (115200)
#define SCI_IRQHandler USART2_IRQHandler
#define RX_BUFF_SIZE 16
#define TX_BUFF_SIZE 512

void SCI_RX_handler(void);
void SCI_TX_handler(void);

//void Init_SCI(void);
void SCI_Init();
void SCI_putc(uint8_t c);
int SCI_puts(const char *str);
int SCI_printf(const char *str, ... );
int SCI_getc(void);
int SCI_checkc(void);
char *SCI_gets(char *s);
void strtofloat(float *f, char* sign, int* t_i, int* t_f);

#endif
