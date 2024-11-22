#ifndef __SPI_H__
#define __SPI_H__

#include "stm32h7xx_hal.h"

#define IMU_SPI      hspi1
#define IMU_DMAC_TX  hdma_spi1_tx
#define IMU_DMAC_RX  hdma_spi1_rx

void spi_Init();
int spi_Transfer(uint8_t *txBuf, uint8_t *rxBuf, int size, uint32_t timeout);
int spi_TransferDMA(uint8_t *txBuf, uint8_t*rxBuf, int size);
void spi_WaitReady();
void spi_RegisterTransferCompleteCallback(void (*callback)());

#endif // __SPI_H__
