#include <stm32h747xx.h>

#include "nvic.h"

void NVIC_Init(void) {

  // priority : 0x00(high)~0xff(low)

  SCB->AIRCR = (SCB->AIRCR & ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)) |
                (0x5FA << SCB_AIRCR_VECTKEY_Pos) | (3 << SCB_AIRCR_PRIGROUP_Pos);

  __COMPILER_BARRIER();

  // debug_console priority 0
  NVIC->IP[USART2_IRQn] = 0x00;
  NVIC->ISER[USART2_IRQn >> 5] = 1 << (USART2_IRQn & 0x1F);

  // IMU kick motion_controll interval priority 1
  NVIC->IP[TIM4_IRQn] = 0x10;
  NVIC->ISER[TIM4_IRQn >> 5] = 1 << (TIM4_IRQn & 0x1F);

  // comm_MPSV priority 2
  NVIC->IP[UART4_IRQn] = 0x40;
  NVIC->ISER[UART4_IRQn >> 5] = 1 << (UART4_IRQn & 0x1F);

  // USB-FS priority 2
  NVIC->IP[OTG_FS_IRQn] = 0x44;
  NVIC->ISER[OTG_FS_IRQn >> 5] = 1 << (OTG_FS_IRQn & 0x1F);

  // SMBUS priority 2
  NVIC->IP[I2C1_EV_IRQn] = 0x48;
  NVIC->ISER[I2C1_EV_IRQn >> 5] = 1 << (I2C1_EV_IRQn & 0x1F);
  NVIC->IP[I2C1_ER_IRQn] = 0x40;
  NVIC->ISER[I2C1_ER_IRQn >> 5] = 1 << (I2C1_ER_IRQn & 0x1F);

  // SPI1 DMA Stream priority 2
  NVIC->IP[DMA1_Stream0_IRQn] = 0x41;
  NVIC->ISER[DMA1_Stream0_IRQn >> 5] = 1 << (DMA1_Stream0_IRQn & 0x1F);
  NVIC->IP[DMA1_Stream1_IRQn] = 0x41;
  NVIC->ISER[DMA1_Stream1_IRQn >> 5] = 1 << (DMA1_Stream1_IRQn & 0x1F);

  // SPI1 EOT -> motion controller
  NVIC->IP[SPI1_IRQn] = 0x80;
  NVIC->ISER[SPI1_IRQn >> 5] = 1 << (SPI1_IRQn & 0x1F);

  // generic timer 100Hz interval priority 128
  NVIC->IP[TIM3_IRQn] = 0xf0;
  NVIC->ISER[TIM3_IRQn >> 5] = 1 << (TIM3_IRQn & 0x1F);
}
