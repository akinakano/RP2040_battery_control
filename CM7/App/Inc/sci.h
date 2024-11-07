#ifndef _SCI_H_
#define _SCI_H_

#include <stdint.h>

#define  SCI_DEBUG_UART     (USART2)

void SCI_RX_handler(void);
void SCI_TX_handler(void);

//void Init_SCI(void);
int32_t SCI_read(int32_t *c);
int32_t SCI_write(int32_t c);
int32_t SCI_putchar(int32_t c);
int32_t SCI_puts(const char *str);
int32_t SCI_printf(const char *str, ... );
int32_t SCI_getchar(void);
int32_t SCI_getchar2(void);
char *SCI_gets(char *s);
void strtofloat(float *f, char* sign, int* t_i, int* t_f);

#endif
