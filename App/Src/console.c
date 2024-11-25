#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <stm32h747xx.h>
#include "console.h"
#include "gpio.h"
#include "main.h"

static volatile uint8_t rx_buff[RX_BUFF_SIZE];
static volatile int rx_tail = 0;
static          int rx_head = 0;
static          uint8_t tx_buff[TX_BUFF_SIZE];
static          int tx_tail = 0;
static volatile int tx_head = 0;

extern uint32_t SystemD2Clock;

void Console_Init(void) { // DebugUART

  // Peripheral clock enable
  RCC->APB1LENR |= RCC_APB1LENR_USART2EN;

  // baudrate : 115200
  // parity   : none
  // word     : 8bit
  // stop     : 1bit
  // flow ctrl: disable
  CONSOLE_UART->CR1 &= ~USART_CR1_UE;
  CONSOLE_UART->CR1 = USART_CR1_TXFEIE | USART_CR1_FIFOEN | USART_CR1_RTOIE | USART_CR1_TE | USART_CR1_RE;
  CONSOLE_UART->CR2 = USART_CR2_RTOEN;
  CONSOLE_UART->CR3 = USART_CR3_RXFTIE | (0b100 << USART_CR3_RXFTCFG_Pos);
  CONSOLE_UART->GTPR = 0; // prescaller x1
  CONSOLE_UART->RTOR = CONSOLE_BAUDRATE * 20 / 1000; // 20ms
  uint32_t usart_div = SystemD2Clock / 2 / CONSOLE_BAUDRATE;
  CONSOLE_UART->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate
  CONSOLE_UART->CR1 |= USART_CR1_UE;

  setbuf(stdout, NULL);
}

void Console_IRQHandler(void) {

  uint32_t isr = CONSOLE_UART->ISR;
  if(isr & USART_ISR_TXFE) {
    while(tx_tail != tx_head) {
      if(!(CONSOLE_UART->ISR & USART_ISR_TXE_TXFNF)) break;
      CONSOLE_UART->TDR = tx_buff[tx_head];
      tx_head = (tx_head + 1) % TX_BUFF_SIZE;
    }
    if(tx_tail == tx_head) CONSOLE_UART->CR1 &= ~USART_CR1_TXFEIE;
  }
  if(isr & (USART_ISR_RXFT | USART_ISR_RTOF)) {
    while(CONSOLE_UART->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t d = (uint8_t)CONSOLE_UART->RDR;
      int rx_tail_tmp = (rx_tail + 1) % RX_BUFF_SIZE;
      if(rx_tail_tmp != rx_head) {
        rx_buff[rx_tail] = d;
        rx_tail = rx_tail_tmp;
      }
    }
    CONSOLE_UART->ICR = USART_ICR_RTOCF;
  }
}

int getcharNonblock(void) {
  if(rx_tail != rx_head) {
    int d = rx_buff[rx_head];
    rx_head = (rx_head + 1) % RX_BUFF_SIZE;
    return d;
  }
  return -1;
}

int getchar(void) {
  int c;
  while((c = getcharNonblock()) < 0);
  if(c == 0x0d) c = '\n';
  return c;
}

int putchar(int c) {

    int txTailTmp = (tx_tail + 1) % TX_BUFF_SIZE;
    if(txTailTmp == tx_head) return -1;
    tx_buff[tx_tail] = c;
    tx_tail = txTailTmp;
    if(!(CONSOLE_UART->CR1 & USART_CR1_TXFEIE)) CONSOLE_UART->CR1 |= USART_CR1_TXFEIE;
    return c;
}

int __io_putchar(int d) {

  if(d == 0x0a) while(putchar(0x0d) < 0);
  while(putchar(d) < 0);
  return d;
}

int __io_getchar(void) {

  return getchar();
}
