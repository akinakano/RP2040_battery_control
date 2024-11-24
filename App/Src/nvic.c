#include <stm32h747xx.h>

#include "nvic.h"

// groupPriority : 0(high) - 15(low)
// subPriority : 0(high) - 15(low)
inline void NVIC_Enable(int irqNo, int groupPriority, int subPriority) {

  NVIC->IP[irqNo] = (groupPriority << 4) | (subPriority & 0x0f);
  NVIC->ISER[irqNo >> 5] = 1 << (irqNo & 0x1F);
}

void NVIC_Init(void) {

  SCB->AIRCR = (SCB->AIRCR & ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)) |
                (0x5FA << SCB_AIRCR_VECTKEY_Pos) | (3 << SCB_AIRCR_PRIGROUP_Pos);

  __COMPILER_BARRIER();

  // debug_console priority 0,0
  NVIC_Enable(USART2_IRQn, 0, 0);

  // IMU kick motion_controll interval priority 1,0
  NVIC_Enable(TIM4_IRQn, 1, 0);

  // comm_MPSV priority 4,0
  NVIC_Enable(UART4_IRQn, 4, 0);

  // USB-FS priority 4,4
  NVIC_Enable(OTG_FS_IRQn, 4, 4);

  // SMBUS priority 4,8
  NVIC_Enable(I2C1_EV_IRQn, 4, 8);
  NVIC_Enable(I2C1_ER_IRQn, 4, 8);

  // SPI1 DMA Stream priority 4,1
  NVIC_Enable(DMA1_Stream0_IRQn, 4, 1);
  NVIC_Enable(DMA1_Stream1_IRQn, 4, 1);

  // SPI1 EOT -> motion controller 8,0
  NVIC_Enable(SPI1_IRQn, 8, 0);

  // generic timer 100Hz interval priority 15, 0
  NVIC_Enable(TIM3_IRQn, 15, 0);
}
