#include <stdint.h>
#include <stdbool.h>
#include <stm32h747xx.h>

#include "tim.h"
#include "comm_apmp.h"
#include "power_control.h"
#include "comm_battery.h"
#include "motion_controller.h"

void TIM3_Init(void) { // 100Hz interval

  RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
  TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM3->PSC = 240 - 1; // 1MHz
  TIM3->ARR = 1000 * 1000 /  100 - 1; // 100Hz
  TIM3->DIER |= TIM_DIER_UIE;
  TIM3->EGR |= TIM_EGR_UG;
}

static uint32_t hz_internal_count = 0;
void TIM3_IRQHandler(void) { // 100Hz

  TIM3->SR &= ~TIM_SR_UIF;

  if(!(hz_internal_count % 2)) { // 50Hz
    Comm_APMP_Tx_Handler();
  }
  if(!(hz_internal_count % 10)) { // 10Hz
    Comm_APMP_ErrorCheckInterval();
    Comm_Battery_Handler();
    PowerControl_Handler();
  }
  if(!(hz_internal_count % 100)) { // 1Hz
    Heartbeat_Handler();
  }
  hz_internal_count++;
}

void TIM4_Init(void) { // motion control interval (high priorirty)

  RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
  TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM4->PSC = 240 - 1; // 1MHz
  TIM4->ARR = 1000 * 1000 / MOTION_CONTROL_FREQ - 1; // 1600Hz
  TIM4->DIER |= TIM_DIER_UIE;
  TIM4->EGR |= TIM_EGR_UG;
}

void TIM4_IRQHandler(void) { // 1600Hz

  TIM4->SR &= ~TIM_SR_UIF;
  MotionControl_IrqHandler();
}
