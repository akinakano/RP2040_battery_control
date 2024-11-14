#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"

void GPIO_Init(void) {

  RCC->AHB4ENR |= (1 << RCC_AHB4ENR_GPIOGEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOEEN_Pos)
                | (1 << RCC_AHB4ENR_GPIODEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOCEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOBEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOAEN_Pos);

  // LED_S1(PA10:AMBER HeartBeat), S2(PG5:Yellow 15V_State) off
  GPIOA->MODER = ((uint32_t)(GPIOA->MODER) & ~GPIO_MODER_MODE10_Msk) | (1 << GPIO_MODER_MODE10_Pos);
  GPIOA->BSRR = (1 << GPIO_BSRR_BS10_Pos);
  GPIOG->MODER = ((uint32_t)(GPIOG->MODER) & ~GPIO_MODER_MODE5_Msk) | (1 << GPIO_MODER_MODE5_Pos);
  GPIOG->BSRR = (1 << GPIO_BSRR_BR5_Pos);

  // POWER-SW input (PC13 internal pull-up)
  GPIOC->MODER = ((uint32_t)(GPIOC->MODER) & ~GPIO_MODER_MODE13_Msk) | (0 << GPIO_MODER_MODE13_Pos);
  GPIOC->PUPDR = ((uint32_t)(GPIOC->PUPDR) & ~GPIO_PUPDR_PUPD13_Msk) | (1 << GPIO_PUPDR_PUPD13_Pos);

  // 15V power-control (PG4 output)
  GPIOG->MODER = ((uint32_t)(GPIOG->MODER) & ~GPIO_MODER_MODE4_Msk) | (1 << GPIO_MODER_MODE4_Pos);

}

//POWER-SW
int GPIO_PWR_Switch(void) {

  return (GPIOC->IDR >> GPIO_IDR_ID13_Pos) & 1;
}

void GPIO_15V(int sw) {

  if(sw) {
    GPIOB->BSRR = GPIO_BSRR_BS2;
  } else {
    GPIOB->BSRR = GPIO_BSRR_BS2 << 16;
  }
}

void Heartbeat_IrqHandler() {

  uint32_t odr = GPIOA->ODR;
  GPIOA->BSRR = ((odr & GPIO_ODR_OD10) << 16) | (~odr & GPIO_ODR_OD10);
}
