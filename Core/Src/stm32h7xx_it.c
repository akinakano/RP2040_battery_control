/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
#include "gpio.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SMBUS_HandleTypeDef hsmbus1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

void NMI_Handler(void) {

  TEST_PF5(1);
  while(1);
}

void HardFault_Handler(void) {

  TEST_PF5(1);
  while(1);
}

void MemManage_Handler(void) {

  TEST_PF5(1);
  while(1);
}

void BusFault_Handler(void) {

  TEST_PF5(1);
  while(1);
}

void UsageFault_Handler(void){

  TEST_PF5(1);
  while(1);
}

void SVC_Handler(void) {

}

void DebugMon_Handler(void) {

}

void PendSV_Handler(void) {

}

void SysTick_Handler(void) {

  HAL_IncTick();
}

void DMA1_Stream0_IRQHandler(void) {

  TEST_PF6(1);
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  TEST_PF6(0);
}

void DMA1_Stream1_IRQHandler(void) {

  TEST_PF6(1);
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  TEST_PF6(0);
}

void I2C1_EV_IRQHandler(void) {

  TEST_PF6(1);
  HAL_SMBUS_EV_IRQHandler(&hsmbus1);
  TEST_PF6(0);
}

void I2C1_ER_IRQHandler(void) {

  TEST_PF6(1);
  HAL_SMBUS_ER_IRQHandler(&hsmbus1);
  TEST_PF6(0);
}

void OTG_FS_IRQHandler(void) {

  TEST_PF6(1);
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  TEST_PF6(0);
}
