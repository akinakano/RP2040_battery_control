#include <stdint.h>
#include <stdbool.h>
#include <stm32h747xx.h>

#include "timer.h"
#include "comm_apmp.h"
#include "power_control.h"
#include "comm_battery.h"
#include "imu_icm42688.h"
#include "gpio.h"

static void TIM3_Init(void);
static void TIM4_Init(void);
static void TIM15_Init(void);

void Timer_Init(void) {

  TIM3_Init();
  TIM4_Init();
  TIM15_Init();
}

static void TIM3_Init(void) { // 100Hz interval

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

static void TIM4_Init(void) { // motion control interval (high priorirty)

  RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
  TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM4->PSC = 240 - 1; // 1MHz
  TIM4->ARR = 1000 * 1000 / MOTION_CONTROL_FREQ - 1; // 1600Hz
  TIM4->DIER |= TIM_DIER_UIE;
  TIM4->EGR |= TIM_EGR_UG;
}

void TIM4_IRQHandler(void) { // 1600Hz

  TIM4->SR &= ~TIM_SR_UIF;

  icm42688_IrqIntervalHandler(); // IMU -> MotionController -> SV
}

static void TIM15_Init(void) { // for IMU Heater control PWM (PE6) PWM_EXIMU_HEATER

  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

  TIM15->PSC = 8 - 1; // 240/8=30MHz
  TIM15->ARR = 30000 * 1000 / EXIMU_HEATER_PWM_FREQ - 1;
  //TIM15->DIER |= TIM_DIER_UIE;
  TIM15->EGR |= TIM_EGR_UG;
  TIM15->CNT = 0;
  TIM15->CCR1 = 0;
  TIM15->CCR2 = (TIM15->ARR) / 20;                      // %5%

  TIM15->CCMR1 = (0 << TIM_CCMR1_CC2S_Pos)              // output
               | (0 << TIM_CCMR1_OC2FE_Pos)             // disable Fast mode
               | (1 << TIM_CCMR1_OC2PE_Pos)             // enable Preload
               | (6 << TIM_CCMR1_OC2M_Pos);             // PWM mode 1

  TIM15->CCER = (TIM_CCER_CC2E);                        // CC2 output enable

  TIM15->BDTR = (TIM_BDTR_MOE);                         // main output enable
  
  TIM15->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

#if 0
static void TIM8_Init(void) { // for IMU External CLK Generate (PB15) SPI_EXIMU_FSCLK

  RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
  TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM4->PSC = 240 - 1; // 1MHz
  TIM4->ARR = 1000 * 1000 / MOTION_CONTROL_FREQ - 1; // 1600Hz
  TIM4->DIER |= TIM_DIER_UIE;
  TIM4->EGR |= TIM_EGR_UG;
}
#endif
