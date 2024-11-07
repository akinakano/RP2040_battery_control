#include <stdint.h>
#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"
#include "uart.h"

void UART2_Init(void) { // DebugUART

    // Peripheral clock enable
    RCC->APB1LENR |= RCC_APB1LENR_USART2EN;
    // PD5      ------> AF7:UART2_TX
    // PD6      ------> AF7:UART2_RX
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk ) | ( 7 << GPIO_AFRL_AFSEL5_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE5_Msk ) | ( 2 << GPIO_MODER_MODE5_Pos ) );
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 7 << GPIO_AFRL_AFSEL6_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE6_Msk ) | ( 2 << GPIO_MODER_MODE6_Pos ) );

    // baudrate : 115200
    // parity   : none
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: disable
    USART2->CR1 &= ~USART_CR1_UE;
    USART2->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->GTPR = 0; // prescaller x1
    uint32_t pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div = pclk / 115200;
    USART2->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    USART2->CR1 |= USART_CR1_UE;
}

void UART4_Init(void){ // MPSV RS-485

    // Peripheral clock enable
    RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
    // PD0     ------> AF8:UART4_RX
    // PD1     ------> AF8:UART4_TX
    // PB14    ------> AF8:UART4_DE
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL0_Msk ) | ( 8 << GPIO_AFRL_AFSEL0_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE0_Msk ) | ( 2 << GPIO_MODER_MODE0_Pos ) );
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL1_Msk ) | ( 8 << GPIO_AFRL_AFSEL1_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE1_Msk ) | ( 2 << GPIO_MODER_MODE1_Pos ) );
    GPIOB->AFR[1] = ( ( GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL14_Msk ) | ( 8 << GPIO_AFRH_AFSEL14_Pos ) );
    GPIOB->MODER = ( ( GPIOB->MODER & ~GPIO_MODER_MODE14_Msk ) | ( 2 << GPIO_MODER_MODE14_Pos ) );

    // baudrate : 3000000
    // parity   : none
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: none
    UART4->CR1 &= ~USART_CR1_UE;
    UART4->CR1 = USART_CR1_FIFOEN | USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_TE | USART_CR1_RE;
    UART4->CR2 = 0;
    UART4->CR3 = USART_CR3_DEM | USART_CR3_EIE;
    UART4->GTPR = 0; // prescaller x1
    uint32_t pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div = pclk / 3000000;
    UART4->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART4->CR1 |= USART_CR1_UE;
}

void UART7_Init(void){  //MP-AP UART (Temporary)

    // Peripheral clock enable
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN;
    // PE7   ------> AF7:UART7_RX
    // PE8   ------> AF7:UART7_TX
    GPIOE->AFR[0] = ( ( GPIOE->AFR[0] & ~GPIO_AFRL_AFSEL7_Msk ) | ( 7 << GPIO_AFRL_AFSEL7_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE7_Msk ) | ( 2 << GPIO_MODER_MODE7_Pos ) );
    GPIOE->AFR[1] = ( ( GPIOE->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk ) | ( 7 << GPIO_AFRH_AFSEL11_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE8_Msk ) | ( 2 << GPIO_MODER_MODE8_Pos ) );

    // baudrate : 115200
    // parity   : none
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: disable
    UART7->CR1 &= ~USART_CR1_UE;
    UART7->CR1 &= ~USART_CR1_UE;
    UART7->CR1 = USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_TE | USART_CR1_RE;
    UART7->CR2 = 0;
    UART7->CR3 = USART_CR3_DEM | USART_CR3_EIE;
    UART7->GTPR = 0; // prescaller x1
    uint32_t pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div = pclk / 115200;
    UART7->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART7->CR1 |= USART_CR1_UE;
}
