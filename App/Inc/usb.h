/**
  ******************************************************************************
  * @file           : usb.h
  * @version        : v1.0
  * @brief          : Header for usbd.c file.
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

#ifndef __USB_H__
#define __USB_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"

#define DEVICE_ID1                   (UID_BASE)
#define DEVICE_ID2                   (UID_BASE + 0x4)
#define DEVICE_ID3                   (UID_BASE + 0x8)
#define APP_RX_DATA_SIZE             2048
#define APP_TX_DATA_SIZE             2048
#define USB_SIZ_STRING_SERIAL        0x1A
#define USBD_VID                     0x054c
#define USBD_LANGID_STRING           1033
#define USBD_MANUFACTURER_STRING     "SONY Group Corporation"
#define USBD_PID_FS                  0xfc02
#define USBD_PRODUCT_STRING_FS       "FCX-2 Virtual ComPort"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"
#define USB_SIZ_BOS_DESC             0x0C

extern USBD_DescriptorsTypeDef FS_Desc;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif // __USB_H__
