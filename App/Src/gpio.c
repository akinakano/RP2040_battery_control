#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"

int Power15V = 0;

void GPIO_Init(void) {

  RCC->AHB4ENR |= (1 << RCC_AHB4ENR_GPIOGEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOEEN_Pos)
                | (1 << RCC_AHB4ENR_GPIODEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOCEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOBEN_Pos)
                | (1 << RCC_AHB4ENR_GPIOAEN_Pos);

  // LED_S1(PA10:AMBER HeartBeat), S2(PG5:Yellow 15V_State) off
  GPIOA->MODER = ((uint32_t)(GPIOA->MODER) & ~GPIO_MODER_MODE10_Msk) | (1 << GPIO_MODER_MODE10_Pos);
  GPIOA->BSRR = GPIO_BSRR_BS10;
  GPIOG->MODER = ((uint32_t)(GPIOG->MODER) & ~GPIO_MODER_MODE5_Msk) | (1 << GPIO_MODER_MODE5_Pos);
  GPIOG->BSRR = GPIO_BSRR_BR5;

  // POWER-SW input (PC13 internal pull-up)
  GPIOC->MODER = ((uint32_t)(GPIOC->MODER) & ~GPIO_MODER_MODE13_Msk) | (0 << GPIO_MODER_MODE13_Pos);
  GPIOC->PUPDR = ((uint32_t)(GPIOC->PUPDR) & ~GPIO_PUPDR_PUPD13_Msk) | (1 << GPIO_PUPDR_PUPD13_Pos);

  // 15V power-control (PG4 output)
  GPIOG->MODER = ((uint32_t)(GPIOG->MODER) & ~GPIO_MODER_MODE4_Msk) | (1 << GPIO_MODER_MODE4_Pos);

}

//POWER-SW
static int GPIO_PWR_Switch(void) {

  return (GPIOC->IDR >> GPIO_IDR_ID13_Pos) & 1;
}

static void GPIO_15V(int sw) {

  if(sw) {
    GPIOB->BSRR = GPIO_BSRR_BS2;
    GPIOG->BSRR = GPIO_BSRR_BS5;
  } else {
    GPIOB->BSRR = GPIO_BSRR_BR2;
    GPIOG->BSRR = GPIO_BSRR_BR5;
  }
}

void PowerControl_Handler() { // 10Hz

  static int powerSW = 1;
  static int count15 = 20;
  static int sw_count = 0;

  int sw = GPIO_PWR_Switch();
  if(sw) {
    sw_count++;
  } else {
    sw_count = 0;
  }
  if(sw_count == 5) {
    powerSW ^= 1;
  }

  if(Power15V != powerSW) {
    if(powerSW) {
      if(count15) {
        count15--;
      } else {
        GPIO_15V(Power15V = powerSW);
      }
    } else {
      count15 = 20;
      GPIO_15V(Power15V = powerSW);
    }
  } else {
    if(count15) count15--;
  }
}

void Heartbeat_Handler() { // 1Hz

  uint32_t odr = GPIOA->ODR;
  GPIOA->BSRR = ((odr & GPIO_ODR_OD10) << 16) | (~odr & GPIO_ODR_OD10);
}
