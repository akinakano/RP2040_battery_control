#include <stdbool.h>
#include <stm32h747xx.h>
#include <stdio.h>

#include "power_control.h"

int Power15V = 0;

void PowerControl_Init(void) {

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
static int Power_Switch(void) {

  return ((GPIOC->IDR >> GPIO_IDR_ID13_Pos) & 1) ^ 1;
}

static void Power15V_Control(int sw) {

  if(sw) {
    GPIOG->BSRR = GPIO_BSRR_BS4;
    GPIOG->BSRR = GPIO_BSRR_BS5;
    printf("Power15V on\n");
  } else {
    GPIOG->BSRR = GPIO_BSRR_BR4;
    GPIOG->BSRR = GPIO_BSRR_BR5;
    printf("Power15V off\n");
  }
}

void PowerControl_Handler() { // 10Hz

  static int powerSW = 1;
  static int powerOnCount = POWER_ON_DELAY;
  static int sw_count = 0;

  int sw = Power_Switch();
  if(sw) {
    sw_count++;
  } else {
    sw_count = 0;
  }
  if(sw_count == 3) powerSW ^= 1;

  if(Power15V != powerSW) {
    if(powerSW) {
      if(powerOnCount) {
        powerOnCount--;
      } else {
        Power15V_Control(Power15V = powerSW);
      }
    } else {
      powerOnCount = POWER_ON_DELAY;
      Power15V_Control(Power15V = powerSW);
    }
  } else {
    if(powerOnCount) powerOnCount--;
  }
}

void Heartbeat_Handler() { // 1Hz

  uint32_t odr = GPIOA->ODR;
  GPIOA->BSRR = ((odr & GPIO_ODR_OD10) << 16) | (~odr & GPIO_ODR_OD10);
}
