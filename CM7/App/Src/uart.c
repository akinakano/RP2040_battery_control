#include <stdint.h>
#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"
#include "uart.h"

void UART4_Init(void){  //MPSV RS485

  return;
    /* Peripheral clock enable */
    //UART2, PD5, PD7
    RCC->APB1LENR |= RCC_APB1LENR_USART2EN;

    /**UART2 GPIO Configuration
    PD5     ------> UART2_TX
    PD6     ------> UART2_RX
    */
    //PD4, PD5, PD6をAlternateFunction7 - UART2に設定
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL4_Msk ) | ( 7 << GPIO_AFRL_AFSEL4_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE4_Msk ) | ( 2 << GPIO_MODER_MODE4_Pos ) );
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk ) | ( 7 << GPIO_AFRL_AFSEL5_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE5_Msk ) | ( 2 << GPIO_MODER_MODE5_Pos ) );
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 7 << GPIO_AFRL_AFSEL6_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE6_Msk ) | ( 2 << GPIO_MODER_MODE6_Pos ) );

    /* UART通信設定 */

    //設定前にUARTをDISABLE
    USART2->CR1 &= ~USART_CR1_UE;

    //UART設定
    // baudrate : 3000000
    // parity   : なし
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: なし

    uint32_t cr1_reg;
    cr1_reg = 0x1UL << USART_CR1_FIFOEN_Pos         //FIFO Enable
            | 0x0UL << USART_CR1_M1_Pos             // M1
            | 0x0UL << USART_CR1_M0_Pos             // M0 = 00 : 8bit
            | 0x0UL << USART_CR1_OVER8_Pos          // 0: oversampling 16
            | 0x0UL << USART_CR1_PCE_Pos            //0: parity無効
            | 0x1UL << USART_CR1_PEIE_Pos           //1: パリティエラーの割り込み有効化
            | 0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos //1: FIFOが空でない状態を検出する割り込み有効化
            | 0x1UL << USART_CR1_TE_Pos             //1: トランスミッタ有効化
            | 0x1UL << USART_CR1_RE_Pos             //1: レシーバ有効化
            ;
    USART2->CR1 = cr1_reg;

    uint32_t cr2_reg;
    cr2_reg = 0x00UL << USART_CR2_STOP_Pos  //00 : STOP 1bit
            ;

    USART2->CR2 = cr2_reg;

    uint32_t cr3_reg;
    cr3_reg = 0x0UL << USART_CR3_RTSE_Pos   //RTS disable
            | 0x0UL << USART_CR3_CTSE_Pos   //CTS disable
            | 0x1UL << USART_CR3_DEM_Pos    //RS485のDE制御 Enable
            | 0x1UL << USART_CR3_EIE_Pos    //エラー割り込み有効化
            ;

    USART2->CR3 = cr3_reg;

    //プリスケーラとbaudrateの設定

    uint32_t gtpr_reg;
    gtpr_reg = 0x0UL << USART_GTPR_PSC_Pos; // prescaller x1

    USART2->GTPR = gtpr_reg;

    uint32_t brr_reg;
    uint32_t pclk;
    pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div;
//     usart_div = pclk / 115200;
    usart_div = pclk / 3000000;
    brr_reg = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    USART2->BRR = brr_reg;

    //設定後にUARTをENABLE
    USART2->CR1 |= USART_CR1_UE;

}

void UART2_Init(void){

    // Peripheral clock enable
    RCC->APB1LENR |= RCC_APB1LENR_USART2EN;

    //PD5, PD6をAF7 - UART2に設定
    // PD5      ------> UART2_TX
    // PD6      ------> UART2_RX
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk ) | ( 7 << GPIO_AFRL_AFSEL5_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE5_Msk ) | ( 2 << GPIO_MODER_MODE5_Pos ) );
    GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 7 << GPIO_AFRL_AFSEL6_Pos ) );
    GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE6_Msk ) | ( 2 << GPIO_MODER_MODE6_Pos ) );

    /* UART通信設定 */
    USART2->CR1 &= ~USART_CR1_UE;

    // baudrate : 115200
    // parity   : none
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: disable
    USART2->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    //プリスケーラとbaudrateの設定
    USART2->GTPR = 0; // prescaller x1
    uint32_t pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div = pclk / 115200;
    USART2->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定

    //設定後にUARTをENABLE
    USART2->CR1 |= USART_CR1_UE;

}

void UART3_Init(void){  //MP-AP

    /* Peripheral clock enable */
    //UART4, PC10, PC11
    RCC->APB1LENR |= RCC_APB1LENR_UART4EN;

    /**UART4 GPIO Configuration
    PC10    ------> UART4_TX
    PC11    ------> UART4_RX
    */
    //PC10, PC11, PA15をAlternateFunction8 - UART4に設定
    GPIOC->AFR[1] = ( ( GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL10_Msk ) | ( 8 << GPIO_AFRH_AFSEL10_Pos ) );
    GPIOC->MODER = ( ( GPIOC->MODER & ~GPIO_MODER_MODE10_Msk ) | ( 2 << GPIO_MODER_MODE10_Pos ) );
    GPIOC->AFR[1] = ( ( GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL11_Msk ) | ( 8 << GPIO_AFRH_AFSEL11_Pos ) );
    GPIOC->MODER = ( ( GPIOC->MODER & ~GPIO_MODER_MODE11_Msk ) | ( 2 << GPIO_MODER_MODE11_Pos ) );
    GPIOA->AFR[1] = ( ( GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL15_Msk ) | ( 8 << GPIO_AFRH_AFSEL15_Pos ) );
    GPIOA->MODER = ( ( GPIOA->MODER & ~GPIO_MODER_MODE15_Msk ) | ( 2 << GPIO_MODER_MODE15_Pos ) );

    /* UART通信設定 */

    //設定前にUARTをDISABLE
    UART4->CR1 &= ~USART_CR1_UE;

    //UART設定
    // baudrate : 3000000
    // parity   : なし
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: なし

    uint32_t cr1_reg;
    cr1_reg = 0x1UL << USART_CR1_FIFOEN_Pos         //FIFO Enable
            | 0x0UL << USART_CR1_M1_Pos             // M1
            | 0x0UL << USART_CR1_M0_Pos             // M0 = 00 : 8bit
            | 0x0UL << USART_CR1_OVER8_Pos          // 0: oversampling 16
            | 0x0UL << USART_CR1_PCE_Pos            //0: parity無効
            | 0x1UL << USART_CR1_PEIE_Pos           //1: パリティエラーの割り込み有効化
            | 0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos //1: FIFOが空でない状態を検出する割り込み有効化
            | 0x1UL << USART_CR1_TE_Pos             //1: トランスミッタ有効化
            | 0x1UL << USART_CR1_RE_Pos             //1: レシーバ有効化
            ;
    UART4->CR1 = cr1_reg;

    uint32_t cr2_reg;
    cr2_reg = 0x00UL << USART_CR2_STOP_Pos  //00 : STOP 1bit
            ;

    UART4->CR2 = cr2_reg;

    uint32_t cr3_reg;
    cr3_reg = 0x0UL << USART_CR3_RTSE_Pos   //RTS disable
            | 0x0UL << USART_CR3_CTSE_Pos   //CTS disable
            | 0x1UL << USART_CR3_DEM_Pos    //RS485のDE制御 Enable
            | 0x1UL << USART_CR3_EIE_Pos    //エラー割り込み有効化
            ;

    UART4->CR3 = cr3_reg;

    //プリスケーラとbaudrateの設定

    uint32_t gtpr_reg;
    gtpr_reg = 0x0UL << USART_GTPR_PSC_Pos; // prescaller x1

    UART4->GTPR = gtpr_reg;

    uint32_t brr_reg;
    uint32_t pclk;
    pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div;
    usart_div = pclk / 115200;
    //usart_div = pclk / 3000000;
    brr_reg = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART4->BRR = brr_reg;

    //設定後にUARTをENABLE
    UART4->CR1 |= USART_CR1_UE;

}

void UART5_Init(void){

    /* Peripheral clock enable */
    //UART5, PB6, PB12
    RCC->APB1LENR |= RCC_APB1LENR_UART5EN;

    /**UART5 GPIO Configuration
    PB6     ------> UART5_TX
    PB12    ------> UART5_RX
    */
    //PB6, PB12をAlternateFunction14, PC8をAlternateFunction8 - UART5に設定
    GPIOB->AFR[0] = ( ( GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 14 << GPIO_AFRL_AFSEL6_Pos ) );
    GPIOB->MODER = ( ( GPIOB->MODER & ~GPIO_MODER_MODE6_Msk ) | ( 2 << GPIO_MODER_MODE6_Pos ) );
    GPIOB->AFR[1] = ( ( GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL12_Msk ) | ( 14 << GPIO_AFRH_AFSEL12_Pos ) );
    GPIOB->MODER = ( ( GPIOB->MODER & ~GPIO_MODER_MODE12_Msk ) | ( 2 << GPIO_MODER_MODE12_Pos ) );
    GPIOC->AFR[1] = ( ( GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk ) | ( 8 << GPIO_AFRH_AFSEL8_Pos ) );
    GPIOC->MODER = ( ( GPIOC->MODER & ~GPIO_MODER_MODE8_Msk ) | ( 2 << GPIO_MODER_MODE8_Pos ) );

    /* UART通信設定 */

    //設定前にUARTをDISABLE
    UART5->CR1 &= ~USART_CR1_UE;

    //UART設定
    // baudrate : 3000000
    // parity   : なし
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: なし

    uint32_t cr1_reg;
    cr1_reg = 0x1UL << USART_CR1_FIFOEN_Pos         //FIFO Enable
            | 0x0UL << USART_CR1_M1_Pos             // M1
            | 0x0UL << USART_CR1_M0_Pos             // M0 = 00 : 8bit
            | 0x0UL << USART_CR1_OVER8_Pos          // 0: oversampling 16
            | 0x0UL << USART_CR1_PCE_Pos            //0: parity無効
            | 0x1UL << USART_CR1_PEIE_Pos           //1: パリティエラーの割り込み有効化
            | 0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos //1: FIFOが空でない状態を検出する割り込み有効化
            | 0x1UL << USART_CR1_TE_Pos             //1: トランスミッタ有効化
            | 0x1UL << USART_CR1_RE_Pos             //1: レシーバ有効化
            ;
    UART5->CR1 = cr1_reg;

    uint32_t cr2_reg;
    cr2_reg = 0x00UL << USART_CR2_STOP_Pos  //00 : STOP 1bit
            ;

    UART5->CR2 = cr2_reg;

    uint32_t cr3_reg;
    cr3_reg = 0x0UL << USART_CR3_RTSE_Pos   //RTS disable
            | 0x0UL << USART_CR3_CTSE_Pos   //CTS disable
            | 0x1UL << USART_CR3_DEM_Pos    //RS485のDE制御 Enable
            | 0x1UL << USART_CR3_EIE_Pos    //エラー割り込み有効化
            ;

    UART5->CR3 = cr3_reg;

    //プリスケーラとbaudrateの設定

    uint32_t gtpr_reg;
    gtpr_reg = 0x0UL << USART_GTPR_PSC_Pos; // prescaller x1

    UART5->GTPR = gtpr_reg;

    uint32_t brr_reg;
    uint32_t pclk;
    pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div;
//     usart_div = pclk / 115200;
    usart_div = pclk / 3000000;
    brr_reg = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART5->BRR = brr_reg;

    //設定後にUARTをENABLE
    UART5->CR1 |= USART_CR1_UE;

}

void UART7_Init(void){

    /* Peripheral clock enable */
    //UART7, PE8, PE7
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN;

    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    //PE8, PE7, PE9をAlternateFunction7 - UART7に設定
    GPIOE->AFR[1] = ( ( GPIOE->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk ) | ( 7 << GPIO_AFRH_AFSEL8_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE8_Msk ) | ( 2 << GPIO_MODER_MODE8_Pos ) );
    GPIOE->AFR[0] = ( ( GPIOE->AFR[0] & ~GPIO_AFRL_AFSEL7_Msk ) | ( 7 << GPIO_AFRL_AFSEL7_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE7_Msk ) | ( 2 << GPIO_MODER_MODE7_Pos ) );
    GPIOE->AFR[1] = ( ( GPIOE->AFR[1] & ~GPIO_AFRH_AFSEL9_Msk ) | ( 7 << GPIO_AFRH_AFSEL9_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE9_Msk ) | ( 2 << GPIO_MODER_MODE9_Pos ) );

    /* UART通信設定 */

    //設定前にUARTをDISABLE
    UART7->CR1 &= ~USART_CR1_UE;

    //UART設定
    // baudrate : 3000000
    // parity   : なし
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: なし

    uint32_t cr1_reg;
    cr1_reg = 0x1UL << USART_CR1_FIFOEN_Pos         //FIFO Enable
            | 0x0UL << USART_CR1_M1_Pos             // M1
            | 0x0UL << USART_CR1_M0_Pos             // M0 = 00 : 8bit
            | 0x0UL << USART_CR1_OVER8_Pos          // 0: oversampling 16
            | 0x0UL << USART_CR1_PCE_Pos            //0: parity無効
            | 0x1UL << USART_CR1_PEIE_Pos           //1: パリティエラーの割り込み有効化
            | 0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos //1: FIFOが空でない状態を検出する割り込み有効化
            | 0x1UL << USART_CR1_TE_Pos             //1: トランスミッタ有効化
            | 0x1UL << USART_CR1_RE_Pos             //1: レシーバ有効化
            ;
    UART7->CR1 = cr1_reg;

    uint32_t cr2_reg;
    cr2_reg = 0x00UL << USART_CR2_STOP_Pos  //00 : STOP 1bit
            ;

    UART7->CR2 = cr2_reg;

    uint32_t cr3_reg;
    cr3_reg = 0x0UL << USART_CR3_RTSE_Pos   //RTS disable
            | 0x0UL << USART_CR3_CTSE_Pos   //CTS disable
            | 0x1UL << USART_CR3_DEM_Pos    //RS485のDE制御 Enable
            | 0x1UL << USART_CR3_EIE_Pos    //エラー割り込み有効化
            ;

    UART7->CR3 = cr3_reg;

    //プリスケーラとbaudrateの設定

    uint32_t gtpr_reg;
    gtpr_reg = 0x0UL << USART_GTPR_PSC_Pos; // prescaller x1

    UART7->GTPR = gtpr_reg;

    uint32_t brr_reg;
    uint32_t pclk;
    pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div;
    usart_div = pclk / 115200;
    //usart_div = pclk / 3000000;
    brr_reg = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART7->BRR = brr_reg;

    //設定後にUARTをENABLE
    UART7->CR1 |= USART_CR1_UE;

}

void UART8_Init(void){

    /* Peripheral clock enable */
    //UART8, PE0, PE1
    RCC->APB1LENR |= RCC_APB1LENR_UART8EN;

    /**UART8 GPIO Configuration
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX
    */
    //PE0, PE1をAlternateFunction8 - UART8に設定
    GPIOE->AFR[0] = ( ( GPIOE->AFR[0] & ~GPIO_AFRL_AFSEL0_Msk ) | ( 8 << GPIO_AFRL_AFSEL0_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE0_Msk ) | ( 2 << GPIO_MODER_MODE0_Pos ) );
    GPIOE->AFR[0] = ( ( GPIOE->AFR[0] & ~GPIO_AFRL_AFSEL1_Msk ) | ( 8 << GPIO_AFRL_AFSEL1_Pos ) );
    GPIOE->MODER = ( ( GPIOE->MODER & ~GPIO_MODER_MODE1_Msk ) | ( 2 << GPIO_MODER_MODE1_Pos ) );
    //GPIOE->PUPDR = (GPIOE->PUPDR & ~GPIO_PUPDR_PUPD0_Msk)  | (0x1UL << GPIO_PUPDR_PUPD0_Pos);
    //GPIOE->PUPDR = (GPIOE->PUPDR & ~GPIO_PUPDR_PUPD1_Msk)  | (0x1UL << GPIO_PUPDR_PUPD1_Pos);

    /* UART通信設定 */

    //設定前にUARTをDISABLE
    UART8->CR1 &= ~USART_CR1_UE;

    //UART設定
    // baudrate : 115200
    // parity   : なし
    // word     : 8bit
    // stop     : 1bit
    // flow ctrl: なし

    uint32_t cr1_reg;
    cr1_reg = 0x1UL << USART_CR1_FIFOEN_Pos         //FIFO Enable
            | 0x0UL << USART_CR1_M1_Pos             // M1
            | 0x0UL << USART_CR1_M0_Pos             // M0 = 00 : 8bit
            | 0x0UL << USART_CR1_OVER8_Pos          // 0: oversampling 16
            | 0x0UL << USART_CR1_PCE_Pos            //0: parity無効
            //| 0x1UL << USART_CR1_PEIE_Pos           //1: パリティエラーの割り込み有効化
            //| 0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos //1: FIFOが空でない状態を検出する割り込み有効化
            | 0x1UL << USART_CR1_TE_Pos             //1: トランスミッタ有効化
            | 0x1UL << USART_CR1_RE_Pos             //1: レシーバ有効化
            ;
    UART8->CR1 = cr1_reg;

    uint32_t cr2_reg;
    cr2_reg = 0x00UL << USART_CR2_STOP_Pos  //00 : STOP 1bit
            ;

    UART8->CR2 = cr2_reg;

    uint32_t cr3_reg;
    cr3_reg = 0x0UL << USART_CR3_RTSE_Pos   //RTS disable
            | 0x0UL << USART_CR3_CTSE_Pos   //CTS disable
            //| 0x1UL << USART_CR3_DEM_Pos    //RS485のDE制御 Enable
            //| 0x1UL << USART_CR3_EIE_Pos    //エラー割り込み有効化
            ;

    UART8->CR3 = cr3_reg;

    //プリスケーラとbaudrateの設定

    uint32_t gtpr_reg;
    gtpr_reg = 0x0UL << USART_GTPR_PSC_Pos; // prescaller x1

    UART8->GTPR = gtpr_reg;

    uint32_t brr_reg;
    uint32_t pclk;
    pclk = 120000000UL; //ToDo: clockから持ってくる。今は120MHz前提。
    uint32_t usart_div;
    usart_div = pclk / 1000000;
    //usart_div = pclk / 3000000;
    brr_reg = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
    UART8->BRR = brr_reg;

    //設定後にUARTをENABLE
    UART8->CR1 |= USART_CR1_UE;

}
