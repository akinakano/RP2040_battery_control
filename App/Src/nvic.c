#include <stm32h747xx.h>
#include <core_cm7.h>

#include "nvic.h"

void NVIC_Init(void) {

  // priority : 0(high)~15(low)
  uint32_t NVIC_PriorityGroup4 = 0x03;
  NVIC_SetPriorityGrouping(NVIC_PriorityGroup4);

  // debug_console priority 0
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  // IMU kick motion_controll interval priority 1
  NVIC_SetPriority(TIM4_IRQn, 1);
  NVIC_EnableIRQ(TIM4_IRQn);

  // comm_MPSV priority 2
  NVIC_SetPriority(UART4_IRQn, 4);
  NVIC_EnableIRQ(UART4_IRQn);

  // USB-FS priority 2
  NVIC_SetPriority(OTG_FS_IRQn, 4);
  NVIC_EnableIRQ(OTG_FS_IRQn);

  // SMBUS priority 2
  NVIC_SetPriority(I2C1_EV_IRQn, 4);
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, 4);
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  // SPI1 DMA Stream priority 2
  NVIC_SetPriority(DMA1_Stream0_IRQn, 4);
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  NVIC_SetPriority(DMA1_Stream1_IRQn, 4);
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  // SPI1 EOT -> motion controller
  NVIC_SetPriority(SPI1_IRQn, 8);
  NVIC_EnableIRQ(SPI1_IRQn);

  // generic timer 100Hz interval priority 128
  NVIC_SetPriority(TIM3_IRQn, 15);
  NVIC_EnableIRQ(TIM3_IRQn);
}
