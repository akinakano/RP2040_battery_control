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
#include "tim.h"
#include "nvic.h"
#include "power_control.h"
#include "console.h"
#include "debug.h"
#include "comm_mpsv.h"
#include "comm_apmp.h"
#include "comm_battery.h"
#include "imu_icm42688.h"
#include "HW_type.h"
#include "vqfInstance.h"


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

void SystemClock_Config(void);
static void MPU_Config(void);

int main(void) {

  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);

  Console_Init();
  PowerControl_Init();
  USB_DEVICE_Init();
  comm_battery_init();
  TIM3_Init();
  TIM4_Init();
  comm_mpsv_Init();
  icm42688_Initialize();
  NVIC_Init();
  __enable_irq();

  while (1) debug_menu();
}

void MPU_Config(void) {

  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void) {

  RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
  uint32_t dummy = RCC->APB4ENR;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN |
                  RCC_AHB4ENR_GPIOEEN |
                  RCC_AHB4ENR_GPIODEN |
                  RCC_AHB4ENR_GPIOCEN |
                  RCC_AHB4ENR_GPIOBEN |
                  RCC_AHB4ENR_GPIOAEN;
  dummy = RCC->AHB4ENR;

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
  HAL_PWREx_ConfigSupply(PWR_SMPS_1V8_SUPPLIES_LDO);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  // Initializes the RCC Oscillators according to the specified parameters
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // Initializes the CPU, AHB and APB buses clocks
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();

  RCC->AHB4ENR |= RCC_AHB4ENR_HSEMEN;
  dummy = RCC->AHB4ENR;
  UNUSED(dummy);

  int32_t timeout = 0xFFFF;;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if(timeout < 0) Error_Handler();
}

void Error_Handler(void) {

  printf("ErrorHandler");
//  __disable_irq();
  while (1);
}
