/**
  ******************************************************************************
  * @file    system_stm32h7xx_dualcore_boot_cm4_cm7.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-Mx Device Peripheral Access Layer System Source File.
  *          This provides system initialization template function is case of
  *          an application using a dual core STM32H7 device where
  *          Cortex-M7 and Cortex-M4 boot are enabled at the FLASH option bytes
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32h7xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock, it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stm32h7xx.h"
#include <math.h>

//#define HSE_VALUE    ((uint32_t)25000000)
//#define CSI_VALUE    ((uint32_t)4000000
//#define HSI_VALUE    ((uint32_t)64000000)

  uint32_t SystemCoreClock = 64000000;
  uint32_t SystemD2Clock = 64000000;
  const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

void SystemInit (void) {

  SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

  if(FLASH_LATENCY_DEFAULT  > (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
  {
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
  }

  RCC->CR |= RCC_CR_HSION;
  RCC->CFGR = 0x00000000;
  RCC->CR &= 0xEAF6ED7FU;
  if(FLASH_LATENCY_DEFAULT  < (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
  {
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
  }

  RCC->D1CFGR = 0x00000000;
  RCC->D2CFGR = 0x00000000;
  RCC->D3CFGR = 0x00000000;
  RCC->PLLCKSELR = 0x02020200;
  RCC->PLLCFGR = 0x01FF0000;
  RCC->PLL1DIVR = 0x01010280;
  RCC->PLL1FRACR = 0x00000000;
  RCC->PLL2DIVR = 0x01010280;
  RCC->PLL2FRACR = 0x00000000;
  RCC->PLL3DIVR = 0x01010280;
  RCC->PLL3FRACR = 0x00000000;
  RCC->CR &= 0xFFFBFFFFU;
  RCC->CIER = 0x00000000;
  EXTI_D2->EMR3 |= 0x4000UL;

  if((DBGMCU->IDCODE & 0xFFFF0000U) < 0x20000000U)
  {
    *((__IO uint32_t*)0x51008108) = 0x000000001U;
  }

  FMC_Bank1_R->BTCR[0] = 0x000030D2;
}
