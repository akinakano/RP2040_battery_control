#include <stm32h747xx.h>
#include <stddef.h>

#include "rs485.h"
#include "HW_type.h"

static volatile uint8_t rx_buff[RS485_RX_BUFF_SIZE];
static volatile int rx_tail = 0;
static          int rx_head = 0;

extern uint32_t SystemD2Clock;

static void (*receiveTimeoutCallback)(void) = NULL;

void rs485_Init(void) {

  RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
  // PD0     ------> AF8:UART4_RX
  // PD1     ------> AF8:UART4_TX
  // PB14    ------> AF8:UART4_DE
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL0_Msk ) | ( 8 << GPIO_AFRL_AFSEL0_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE0_Msk ) | ( 2 << GPIO_MODER_MODE0_Pos ) );
  GPIOD->PUPDR = ((uint32_t)(GPIOD->PUPDR) & ~GPIO_PUPDR_PUPD0_Msk) | (1 << GPIO_PUPDR_PUPD0_Pos);
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL1_Msk ) | ( 8 << GPIO_AFRL_AFSEL1_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE1_Msk ) | ( 2 << GPIO_MODER_MODE1_Pos ) );
#ifdef FCX_1
  // PA15    ------> AF8:UART4_DE
  GPIOA->AFR[1] = ( ( GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL15_Msk ) | ( 8 << GPIO_AFRH_AFSEL15_Pos ) );
  GPIOA->MODER = ( ( GPIOA->MODER & ~GPIO_MODER_MODE15_Msk ) | ( 2 << GPIO_MODER_MODE15_Pos ) );
#else
  // PB14    ------> AF8:UART4_DE
  GPIOB->AFR[1] = ( ( GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL14_Msk ) | ( 8 << GPIO_AFRH_AFSEL14_Pos ) );
  GPIOB->MODER = ( ( GPIOB->MODER & ~GPIO_MODER_MODE14_Msk ) | ( 2 << GPIO_MODER_MODE14_Pos ) );
#endif

  // baudrate : 3000000
  // parity   : none
  // word     : 8bit
  // stop     : 1bit
  // flow ctrl: none
  RS485_UART->CR1 &= ~USART_CR1_UE;
  RS485_UART->CR1 = USART_CR1_FIFOEN | USART_CR1_PEIE | USART_CR1_RTOIE | USART_CR1_TE | USART_CR1_RE;
  RS485_UART->CR2 = USART_CR2_RTOEN;
  RS485_UART->CR3 = USART_CR3_RXFTIE | (0b100 << USART_CR3_RXFTCFG_Pos) | USART_CR3_DEM;
  RS485_UART->GTPR = 0; // prescaller x1
  RS485_UART->RTOR = RS485_BAUDRATE * 50 / 1000 / 1000; // 50us
  uint32_t usart_div = SystemD2Clock / 2 / RS485_BAUDRATE;
  RS485_UART->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate
  RS485_UART->CR1 |= USART_CR1_UE;
}

void rs485_send(uint8_t *buf, int len) {

  int timeout_count = 10000;
  for(int i = 0; i < len; i++) {
    while(!(RS485_UART->ISR & USART_ISR_TXE_TXFNF) && timeout_count--);
    RS485_UART->TDR = buf[i];
  }
}

void RS485_IRQHandler() {

  uint32_t isr = RS485_UART->ISR;
  if(isr & (USART_ISR_RXFT | USART_ISR_RTOF)) {
    while(RS485_UART->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t d = (uint8_t)RS485_UART->RDR;
      int rx_tail_tmp = (rx_tail + 1) % RS485_RX_BUFF_SIZE;
      if(rx_tail_tmp != rx_head) {
        rx_buff[rx_tail] = d;
        rx_tail = rx_tail_tmp;
      }
    }
    RS485_UART->ICR = USART_ICR_RTOCF;
    if(receiveTimeoutCallback && (isr & USART_ISR_RTOF)) receiveTimeoutCallback();
  }
}

void rs485_register_receive_timeout_callback(void (*func)(void)) {

  receiveTimeoutCallback = func;
}

int rs485_readByte(void) {

  if(rx_tail != rx_head) {
    int d = rx_buff[rx_head];
    rx_head = (rx_head + 1) % RS485_RX_BUFF_SIZE;
    return d;
  }
  return -1;
}

int rs485_receivedLength(void) {

  return (rx_tail + RS485_RX_BUFF_SIZE - rx_head) % RS485_RX_BUFF_SIZE;
}
