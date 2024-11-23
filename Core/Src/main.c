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

#include "system.h"
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

void Error_Handler(void) {

  TEST_PF5(1);
  while(1);
}
