
#include "stm32h7xx_hal.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

SPI_HandleTypeDef IMU_SPI;
DMA_HandleTypeDef IMU_DMAC_RX;
DMA_HandleTypeDef IMU_DMAC_TX;

static void (*transferCompleteCallback)(void) = (void (*)())NULL;
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAError(DMA_HandleTypeDef *hdma);

void spi_Init() {

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* SPI1_RX DMA Init */
  IMU_DMAC_RX.Instance = DMA1_Stream0;
  IMU_DMAC_RX.Init.Request = DMA_REQUEST_SPI1_RX;
  IMU_DMAC_RX.Init.Direction = DMA_PERIPH_TO_MEMORY;
  IMU_DMAC_RX.Init.PeriphInc = DMA_PINC_DISABLE;
  IMU_DMAC_RX.Init.MemInc = DMA_MINC_ENABLE;
  IMU_DMAC_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  IMU_DMAC_RX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  IMU_DMAC_RX.Init.Mode = DMA_NORMAL;
  IMU_DMAC_RX.Init.Priority = DMA_PRIORITY_LOW;
  IMU_DMAC_RX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  IMU_DMAC_RX.Lock = HAL_UNLOCKED;
  if(HAL_DMA_Init(&IMU_DMAC_RX) != HAL_OK) Error_Handler();
  IMU_SPI.hdmarx = &IMU_DMAC_RX;
  IMU_DMAC_RX.Parent = &IMU_SPI;

  /* SPI1_TX DMA Init */
  IMU_DMAC_TX.Instance = DMA1_Stream1;
  IMU_DMAC_TX.Init.Request = DMA_REQUEST_SPI1_TX;
  IMU_DMAC_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
  IMU_DMAC_TX.Init.PeriphInc = DMA_PINC_DISABLE;
  IMU_DMAC_TX.Init.MemInc = DMA_MINC_ENABLE;
  IMU_DMAC_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  IMU_DMAC_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  IMU_DMAC_TX.Init.Mode = DMA_NORMAL;
  IMU_DMAC_TX.Init.Priority = DMA_PRIORITY_LOW;
  IMU_DMAC_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  IMU_DMAC_TX.Lock = HAL_UNLOCKED;
  if(HAL_DMA_Init(&IMU_DMAC_TX) != HAL_OK) Error_Handler();
  IMU_SPI.hdmatx = &IMU_DMAC_TX;
  IMU_DMAC_TX.Parent = &IMU_SPI;

  IMU_SPI.Instance = SPI1;
  IMU_SPI.State     = HAL_SPI_STATE_READY;

  SPI1->CR1 &= SPI_CR1_MASRX_Msk;
  SPI1->CFG1 = (6 << SPI_CFG1_MBR_Pos) | (0 << SPI_CFG1_CRCEN_Pos) | (7 << SPI_CFG1_CRCSIZE_Pos) |
               (0 << SPI_CFG1_FTHLV_Pos) | (7 << SPI_CFG1_DSIZE_Pos);
  SPI1->CFG2 = SPI_CFG2_SSOM | SPI_CFG2_SSOE | SPI_CFG2_MASTER;
  SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
}

void spi_RegisterTransferCompleteCallback(void (*callback)()) {

  transferCompleteCallback = callback;
}

int spi_Transfer(uint8_t *txBuf, uint8_t *rxBuf, int size, uint32_t timeout) {

  if (IMU_SPI.State != HAL_SPI_STATE_READY) return HAL_BUSY;
  IMU_SPI.State = HAL_SPI_STATE_BUSY_TX;

  int txCount = size;
  int rxCount = size;
  SPI1->CFG2 &= ~SPI_CFG2_COMM;
  if(!rxBuf) {
    SPI1->CFG2 |= SPI_CFG2_COMM_0;
    rxCount = 0;
  }
  SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_TSIZE) | (size << SPI_CR2_TSIZE_Pos);
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_CSTART;

  int error = 0;
  uint32_t tickstart = HAL_GetTick();
  while(txCount || rxCount) {
    if((SPI1->SR & SPI_SR_TXP) && txCount && (rxCount < txCount + 16)) {
      *(__IO uint8_t *)&SPI1->TXDR = *txBuf++;
      txCount--;
    }
    if(rxCount) {
      if(SPI1->SR & SPI_SR_RXP) {
        *rxBuf++ = *(__IO uint8_t *)&SPI1->RXDR;
        rxCount--;
      } else {
        if((HAL_GetTick() - tickstart) >= timeout) {
          error = HAL_SPI_ERROR_TIMEOUT;
          break;
        }
      }
    }
  }
  tickstart = HAL_GetTick();
  while(!(SPI1->SR & SPI_SR_EOT)) {
    if((HAL_GetTick() - tickstart) >= timeout * size) {
      error = HAL_SPI_ERROR_TIMEOUT;
      break;
    }
  }

  SPI1->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_UDRC;
  SPI1->CR1 &= ~SPI_CR1_SPE;
  IMU_SPI.State = HAL_SPI_STATE_READY;
  if(error == HAL_SPI_ERROR_TIMEOUT) return HAL_TIMEOUT;
  return HAL_OK;
}

int spi_TransferDMA(uint8_t *txBuf, uint8_t*rxBuf, int size) {

  if (IMU_SPI.State != HAL_SPI_STATE_READY) return HAL_BUSY;
  IMU_SPI.State = HAL_SPI_STATE_BUSY_TX_RX;

  SPI1->CFG2 &= ~SPI_CFG2_COMM;
  SPI1->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  IMU_SPI.hdmarx->XferHalfCpltCallback = NULL;
  IMU_SPI.hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
  IMU_SPI.hdmarx->XferErrorCallback = SPI_DMAError;
  IMU_SPI.hdmarx->XferAbortCallback = NULL;
  if (HAL_OK != HAL_DMA_Start_IT(IMU_SPI.hdmarx, (uint32_t)&SPI1->RXDR, (uint32_t)rxBuf, size)) {
    IMU_SPI.State = HAL_SPI_STATE_READY;
    return HAL_ERROR;
  }
  SPI1->CFG1 |= SPI_CFG1_RXDMAEN;

  IMU_SPI.hdmatx->XferHalfCpltCallback = NULL;
  IMU_SPI.hdmatx->XferCpltCallback     = NULL;
  IMU_SPI.hdmatx->XferAbortCallback    = NULL;
  IMU_SPI.hdmatx->XferErrorCallback    = SPI_DMAError;
  if (HAL_OK != HAL_DMA_Start_IT(IMU_SPI.hdmatx, (uint32_t)txBuf, (uint32_t)&SPI1->TXDR, size)) {
    HAL_DMA_Abort(IMU_SPI.hdmarx);
    IMU_SPI.State = HAL_SPI_STATE_READY;
    return HAL_ERROR;
  }

  SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_TSIZE) | (size << SPI_CR2_TSIZE_Pos);
  SPI1->CFG1 |= SPI_CFG1_TXDMAEN;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_CSTART;

  return HAL_OK;  return 0;
}

static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma) {

  SPI1->IER |= SPI_IER_EOTIE;
}

static void SPI_CloseTransfer() {

  SPI1->IFCR |= SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_TIFREC;
  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->IER &= ~(SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | \
                              SPI_IT_FRE | SPI_IT_MODF);
  SPI1->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
}

static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (HAL_DMA_GetError(hdma) != HAL_DMA_ERROR_FE) {
    SPI_CloseTransfer();
    hspi->State = HAL_SPI_STATE_READY;
  }
}

void SPI1_IRQHandler(void) {

  if(SPI1->SR & SPI_SR_EOT) {
    SPI1->IFCR |= SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC;
    SPI1->IER &= ~SPI_IER_EOTIE;
    SPI_CloseTransfer();
    IMU_SPI.State = HAL_SPI_STATE_READY;
    if(transferCompleteCallback) transferCompleteCallback();
  }
}
