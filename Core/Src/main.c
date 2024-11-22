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

  // SystemClock Config
  uint32_t common_system_clock = HAL_RCC_GetSysClockFreq() >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE)>> RCC_D1CFGR_D1CPRE_Pos]) & 0x1FU);
  SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)>> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
  SystemCoreClock = common_system_clock;
  HAL_InitTick(TICK_INT_PRIORITY);

  RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
  uint32_t dummy = RCC->APB4ENR;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN |
                  RCC_AHB4ENR_GPIOFEN |
                  RCC_AHB4ENR_GPIOEEN |
                  RCC_AHB4ENR_GPIODEN |
                  RCC_AHB4ENR_GPIOCEN |
                  RCC_AHB4ENR_GPIOBEN |
                  RCC_AHB4ENR_GPIOAEN;
  dummy = RCC->AHB4ENR;

  PWR->CR3 = ((uint32_t)PWR->CR3 & ~PWR_SUPPLY_CONFIG_MASK) | PWR_SMPS_1V8_SUPPLIES_LDO;

  uint32_t tickstart = HAL_GetTick ();
  while((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) != PWR_CSR1_ACTVOSRDY) {
    if((HAL_GetTick () - tickstart) > 1000U) Error_Handler();
  }

  PWR->D3CR = ((uint32_t)PWR->D3CR & PWR_D3CR_VOS) | PWR_REGULATOR_VOLTAGE_SCALE1;
  dummy = PWR->D3CR;
  SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;
  dummy = SYSCFG->PWRCR;
  while((PWR->D3CR & PWR_D3CR_VOSRDY) != PWR_D3CR_VOSRDY);

  // Initializes the RCC Oscillators according to the specified parameters
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();

  RCC->AHB4ENR |= RCC_AHB4ENR_HSEMEN;
  dummy = RCC->AHB4ENR;
  UNUSED(dummy);

  int32_t timeout = 0xFFFF;;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if(timeout < 0) Error_Handler();

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 400;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) Error_Handler();
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) Error_Handler();
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  RCC->D2CCIP2R = ((uint32_t)(RCC->D2CCIP2R) & ~RCC_D2CCIP2R_I2C123SEL) | RCC_I2C123CLKSOURCE_D2PCLK1;

}

void Error_Handler(void) {

  TEST_PF5(1);
  while(1);
}
