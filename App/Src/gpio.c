
#include <stm32h747xx.h>

#include "gpio.h"

void GPIO_Init() {

// console
  // PD5      ------> AF7:UART2_TX
  // PD6      ------> AF7:UART2_RX
  GPIOD->AFR[0] = (GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk) | (7 << GPIO_AFRL_AFSEL5_Pos);
  GPIOD->MODER  = (GPIOD->MODER  & ~GPIO_MODER_MODE5_Msk) | (2 << GPIO_MODER_MODE5_Pos);
  GPIOD->AFR[0] = (GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk) | (7 << GPIO_AFRL_AFSEL6_Pos);
  GPIOD->MODER  = (GPIOD->MODER  & ~GPIO_MODER_MODE6_Msk) | (2 << GPIO_MODER_MODE6_Pos);
  GPIOD->PUPDR  = (GPIOD->PUPDR  & ~GPIO_PUPDR_PUPD6_Msk) | (1 << GPIO_PUPDR_PUPD6_Pos);

// RS-485
  // PD0     ------> AF8:UART4_RX
  // PD1     ------> AF8:UART4_TX
  // PB14    ------> AF8:UART4_DE
  GPIOD->AFR[0] = (GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL0_Msk) | (8 << GPIO_AFRL_AFSEL0_Pos);
  GPIOD->MODER  = (GPIOD->MODER  & ~GPIO_MODER_MODE0_Msk) | (2 << GPIO_MODER_MODE0_Pos);
  GPIOD->PUPDR  = (GPIOD->PUPDR  & ~GPIO_PUPDR_PUPD0_Msk) | (1 << GPIO_PUPDR_PUPD0_Pos);
  GPIOD->AFR[0] = (GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL1_Msk) | (8 << GPIO_AFRL_AFSEL1_Pos);
  GPIOD->MODER  = (GPIOD->MODER  & ~GPIO_MODER_MODE1_Msk) | (2 << GPIO_MODER_MODE1_Pos);
  // PB14    ------> AF8:UART4_DE
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL14_Msk) | (8 << GPIO_AFRH_AFSEL14_Pos);
  GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE14_Msk) | (2 << GPIO_MODER_MODE14_Pos);

// SPI1
  // PA15    ------> SPI1_CS
  // PB3     ------> SPI1_SCK
  // PB4     ------> SPI1_MISO
  // PB5     ------> SPI1_MOSI
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL15_Msk) | (5 << GPIO_AFRH_AFSEL15_Pos);
  GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE15_Msk) | (2 << GPIO_MODER_MODE15_Pos);
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL3_Msk) | (5 << GPIO_AFRL_AFSEL3_Pos);
  GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE3_Msk) | (2 << GPIO_MODER_MODE3_Pos);
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL4_Msk) | (5 << GPIO_AFRL_AFSEL4_Pos);
  GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE4_Msk) | (2 << GPIO_MODER_MODE4_Pos);
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk) | (5 << GPIO_AFRL_AFSEL5_Pos);
  GPIOB->MODER  = (GPIOB->MODER  & ~GPIO_MODER_MODE5_Msk) | (2 << GPIO_MODER_MODE5_Pos);

// USB-FS
  // PA9     ------> USB_OTG_FS_VBUS
  // PA11     ------> USB_OTG_FS_DM
  // PA12     ------> USB_OTG_FS_DP
  GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE9_Msk)  | (0 << GPIO_MODER_MODE9_Pos);
  GPIOA->PUPDR  = (GPIOA->PUPDR  & ~GPIO_PUPDR_PUPD9_Msk)  | (0 << GPIO_PUPDR_PUPD9_Pos);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL11_Msk) | (10 << GPIO_AFRH_AFSEL11_Pos);
  GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE11_Msk) | (2 << GPIO_MODER_MODE11_Pos);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL12_Msk) | (10 << GPIO_AFRH_AFSEL11_Pos);
  GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE12_Msk) | (2 << GPIO_MODER_MODE11_Pos);

// TEST_PF9 GPIO_PF9
  // PF5     ------> GPIO out
  // PF6     ------> GPIO out
  // PF9     ------> GPIO out
  GPIOF->MODER  = (GPIOF->MODER  & ~GPIO_MODER_MODE5_Msk) | (1 << GPIO_MODER_MODE5_Pos);
  GPIOF->BSRR   = GPIO_BSRR_BR5;
  GPIOF->MODER  = (GPIOF->MODER  & ~GPIO_MODER_MODE6_Msk) | (1 << GPIO_MODER_MODE6_Pos);
  GPIOF->BSRR   = GPIO_BSRR_BR6;
  GPIOF->MODER  = (GPIOF->MODER  & ~GPIO_MODER_MODE9_Msk) | (1 << GPIO_MODER_MODE9_Pos);
  GPIOF->BSRR   = GPIO_BSRR_BR9;
}

void TEST_PF5(int d) {

  if(d == 1) {
    GPIOF->BSRR = GPIO_BSRR_BS5;
  } else if(d == 0) {
    GPIOF->BSRR = GPIO_BSRR_BR5;
  } else {
    uint32_t odr = GPIOF->ODR;
    GPIOF->BSRR = ((odr & GPIO_ODR_OD5) << 16) | (~odr & GPIO_ODR_OD5);
  }
}

void TEST_PF6(int d) {

  if(d == 1) {
    GPIOF->BSRR = GPIO_BSRR_BS6;
  } else if(d == 0) {
    GPIOF->BSRR = GPIO_BSRR_BR6;
  } else {
    uint32_t odr = GPIOF->ODR;
    GPIOF->BSRR = ((odr & GPIO_ODR_OD6) << 16) | (~odr & GPIO_ODR_OD6);
  }
}

void TEST_PF9(int d) {

  if(d == 1) {
    GPIOF->BSRR = GPIO_BSRR_BS9;
  } else if(d == 0) {
    GPIOF->BSRR = GPIO_BSRR_BR9;
  } else {
    uint32_t odr = GPIOF->ODR;
    GPIOF->BSRR = ((odr & GPIO_ODR_OD9) << 16) | (~odr & GPIO_ODR_OD9);
  }
}
