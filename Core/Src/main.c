/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "usbd_core.h"
#include "usb.h"
#include "timer.h"
#include "nvic.h"
#include "power_control.h"
#include "console.h"
#include "debug.h"
#include "gpio.h"
#include "comm_mpsv.h"
#include "comm_apmp.h"
#include "comm_battery.h"
#include "imu_icm42688.h"
#include "motion_controller.h"
#include "HW_type.h"
#include "vqfInstance.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

static void SystemConfig(void);

int main(void) {

  SystemConfig();

  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);

  GPIO_Init();
  Console_Init();
  PowerControl_Init();
  USB_DEVICE_Init();
  comm_battery_init();
  Timer_Init();
  comm_mpsv_Init();
  icm42688_Init();
  MotionControl_Init();
  NVIC_Init();
  __enable_irq();

  while (1) debug_menu();
}

static void SystemConfig(void) {

  // MPU Config
  __DMB();
  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
  MPU->CTRL = 0;

  MPU->RNR = MPU_REGION_NUMBER0;
  MPU->RASR &= ~MPU_RASR_ENABLE_Msk;
  MPU->RBAR = 0x00000000;
  MPU->RASR = (MPU_INSTRUCTION_ACCESS_DISABLE << MPU_RASR_XN_Pos)   |
              (MPU_TEX_LEVEL0                 << MPU_RASR_TEX_Pos)  |
              (MPU_ACCESS_SHAREABLE           << MPU_RASR_S_Pos)    |
              (MPU_ACCESS_NOT_CACHEABLE       << MPU_RASR_C_Pos)    |
              (MPU_ACCESS_NOT_BUFFERABLE      << MPU_RASR_B_Pos)    |
              (0x87                           << MPU_RASR_SRD_Pos)  |
              (MPU_REGION_SIZE_4GB            << MPU_RASR_SIZE_Pos) |
              (MPU_REGION_ENABLE             << MPU_RASR_ENABLE_Pos);

  MPU->CTRL = MPU_PRIVILEGED_DEFAULT | MPU_CTRL_ENABLE_Msk;
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
  __DSB();
  __ISB();

  // Power config
  PWR->CR3 = ((uint32_t)PWR->CR3 & ~PWR_SUPPLY_CONFIG_MASK) | PWR_SMPS_1V8_SUPPLIES_LDO;

  uint32_t tickstart = HAL_GetTick();
  while((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) != PWR_CSR1_ACTVOSRDY) {
    if(HAL_GetTick () - tickstart > 1000) Error_Handler();
  }

  PWR->D3CR = ((uint32_t)PWR->D3CR & PWR_D3CR_VOS) | PWR_REGULATOR_VOLTAGE_SCALE1;
  uint32_t dummy = PWR->D3CR;
  SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;
  dummy = SYSCFG->PWRCR;
  while((PWR->D3CR & PWR_D3CR_VOSRDY) != PWR_D3CR_VOSRDY);

  // SystemClock Config
  SystemD2Clock = HSI_VALUE; // internal Ring Osc 64MHz
  SystemCoreClock = HSI_VALUE;
  HAL_InitTick(TICK_INT_PRIORITY);

  // External XTAL 25MHz
  RCC->CR |= RCC_CR_HSEON;
  tickstart = HAL_GetTick();
  while(!(RCC->CR & RCC_CR_HSERDY)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  // PLL1
  // HSE: 25MHz / 5 x 192 = 960MHz
  //     960MHz / 2  -> P1 -> 480MHz -> SYSCLK -> 1/2 -> 1/2 -> APB/AHB PeriCLK
  //     960MHz / 5  -> Q1 -> 192MHz -> SPI1,2,3
  //     960MHz / 2  -> R1 -> 480MHz
  RCC->CR &= ~RCC_CR_PLL1ON;
  tickstart = HAL_GetTick();
  while(RCC->CR & RCC_CR_PLL1RDY) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }
  RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC_Msk | RCC_PLLCKSELR_DIVM1)) |
                   RCC_PLLSOURCE_HSE | (5 << RCC_PLLCKSELR_DIVM1_Pos);
  RCC->PLL1DIVR = ((192 - 1) << RCC_PLL1DIVR_N1_Pos) |
                  ((  2 - 1) << RCC_PLL1DIVR_P1_Pos) |
                  ((  5 - 1) << RCC_PLL1DIVR_Q1_Pos) |
                  ((  2 - 1) << RCC_PLL1DIVR_R1_Pos);
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1FRACEN;
  RCC->PLL1FRACR &= ~RCC_PLL1FRACR_FRACN1;
  RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLL1RGE) | RCC_PLLCFGR_PLL1RGE_2;
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN | RCC_PLLCFGR_PLL1FRACEN;
  RCC->CR |= RCC_CR_PLL1ON;
  tickstart = HAL_GetTick();
  while(!(RCC->CR & RCC_CR_PLL1RDY)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  // PLL2
  // HSE: 25MHz / 2  x  40 = 500MHz
  //     500MHz / 5  -> P2 -> 100MHz
  //     500MHz / 5  -> Q2 -> 100MHz
  //     500MHz / 5  -> R2 -> 100MHz -> SDMMC1,2
  RCC->CR &= ~RCC_CR_PLL2ON;
  tickstart = HAL_GetTick();
  while(RCC->CR & RCC_CR_PLL2RDY) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }
  RCC->PLLCKSELR = (RCC->PLLCKSELR & ~RCC_PLLCKSELR_DIVM2) | (2 << RCC_PLLCKSELR_DIVM2_Pos);
  RCC->PLL2DIVR = (( 40 - 1) << RCC_PLL2DIVR_N2_Pos) |
                  ((  5 - 1) << RCC_PLL2DIVR_P2_Pos) |
                  ((  5 - 1) << RCC_PLL2DIVR_Q2_Pos) |
                  ((  5 - 1) << RCC_PLL2DIVR_R2_Pos);
  RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLL2RGE) | RCC_PLLCFGR_PLL2RGE_0;
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL2VCOSEL;
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL2FRACEN;
  RCC->PLL2FRACR &= ~RCC_PLL2FRACR_FRACN2;
  RCC->PLLCFGR |= RCC_PLLCFGR_PLL2FRACEN;
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVP2EN |RCC_PLLCFGR_DIVR2EN ;
  RCC->CR |= RCC_CR_PLL2ON;
  tickstart = HAL_GetTick();
  while(!(RCC->CR & RCC_CR_PLL2RDY)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  // PLL3
  // HSE: 25MHz /  5 x 96  = 480MHz
  //     480MHz /  2 -> P3 -> 240MHz
  //     480MHz / 10 -> Q3 ->  48MHz -> USB-FS
  //     480MHz /  2 -> R3 -> 240MHz
  RCC->CR &= ~RCC_CR_PLL3ON;
  tickstart = HAL_GetTick();
  while(RCC->CR & RCC_CR_PLL3RDY) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }
  RCC->PLLCKSELR = (RCC->PLLCKSELR & ~RCC_PLLCKSELR_DIVM3) | (5 << RCC_PLLCKSELR_DIVM3_Pos);
  RCC->PLL3DIVR = (( 96 - 1) << RCC_PLL3DIVR_N3_Pos) |
                  ((  2 - 1) << RCC_PLL3DIVR_P3_Pos) |
                  (( 10 - 1) << RCC_PLL3DIVR_Q3_Pos) |
                  ((  2 - 1) << RCC_PLL3DIVR_R3_Pos);
  RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLL3RGE) | RCC_PLLCFGR_PLL3RGE_0;
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL3VCOSEL;
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL3FRACEN;
  RCC->PLL3FRACR &= ~RCC_PLL3FRACR_FRACN3;
  RCC->PLLCFGR |= RCC_PLLCFGR_PLL3FRACEN;
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVQ3EN;
  RCC->CR |= RCC_CR_PLL3ON;
  tickstart = HAL_GetTick();
  while(!(RCC->CR & RCC_CR_PLL3RDY)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC->D1CFGR = (RCC->D1CFGR & ~RCC_D1CFGR_D1PPRE) | RCC_APB3_DIV2;
  RCC->D2CFGR = (RCC->D2CFGR & ~RCC_D2CFGR_D2PPRE1) | RCC_APB1_DIV2;
  RCC->D2CFGR = (RCC->D2CFGR & ~RCC_D2CFGR_D2PPRE2) | RCC_APB2_DIV2;
  RCC->D3CFGR = (RCC->D3CFGR & ~RCC_D3CFGR_D3PPRE) | RCC_APB4_DIV2;
  RCC->D1CFGR = (RCC->D1CFGR & ~RCC_D1CFGR_HPRE) | RCC_HCLK_DIV2;
  RCC->D1CFGR = (RCC->D1CFGR & ~RCC_D1CFGR_D1CPRE) | RCC_SYSCLK_DIV1;

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
  tickstart = HAL_GetTick();
  while((RCC->CFGR & RCC_CFGR_SWS) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  uint32_t common_system_clock = HAL_RCC_GetSysClockFreq() >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos]) & 0x1FU);

  SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE) >> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
  SystemCoreClock = common_system_clock;
  HAL_InitTick(uwTickPrio);

  tickstart = HAL_GetTick();
  while(!(RCC->CR & RCC_CR_D2CKRDY)) {
    if(HAL_GetTick() - tickstart > 100) Error_Handler();
  }

  // SystemController Enable
  RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
  dummy = RCC->APB4ENR;

  // clock source selector
  RCC->D1CCIPR = RCC_D1CCIPR_SDMMCSEL; // PLL2R(100MHz)->SDMMC
  RCC->D2CCIP2R = 0;
  RCC->D2CCIP2R = RCC_D2CCIP2R_USBSEL_1; // PLL3Q(48MHz)->USB-FS

  // GPIO clk
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN |
                  RCC_AHB4ENR_GPIOFEN |
                  RCC_AHB4ENR_GPIOEEN |
                  RCC_AHB4ENR_GPIODEN |
                  RCC_AHB4ENR_GPIOCEN |
                  RCC_AHB4ENR_GPIOBEN |
                  RCC_AHB4ENR_GPIOAEN;
  dummy = RCC->AHB4ENR;

  // SPI1 clk
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  dummy = RCC->APB2ENR;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  dummy = RCC->AHB1ENR;

  // USB clk
  RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGHSEN;
  dummy = RCC->AHB1ENR;

  // HSM clk
  RCC->AHB4ENR |= RCC_AHB4ENR_HSEMEN;
  dummy = RCC->AHB4ENR;

  UNUSED(dummy);
}

void Error_Handler(void) {

  TEST_PF5(1);
  while(1);
}
