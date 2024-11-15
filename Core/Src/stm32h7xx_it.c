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

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SMBUS_HandleTypeDef hsmbus1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;

void NMI_Handler(void) {

   while (1);
}

void HardFault_Handler(void) {

  while (1);
}

void MemManage_Handler(void) {

  while (1);
}

void BusFault_Handler(void) {

  while (1);
}

void UsageFault_Handler(void){

  while (1);
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

  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA1_Stream1_IRQHandler(void) {

  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void I2C1_EV_IRQHandler(void) {

  HAL_SMBUS_EV_IRQHandler(&hsmbus1);
}

void I2C1_ER_IRQHandler(void) {

 HAL_SMBUS_ER_IRQHandler(&hsmbus1);
}

void SPI1_IRQHandler(void) {

  HAL_SPI_IRQHandler(&hspi1);
}

void OTG_FS_IRQHandler(void) {

  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

}
