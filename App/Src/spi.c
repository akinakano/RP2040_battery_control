
#include "stm32h7xx_hal.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

typedef enum {
  SPI_State_Ready,
  SPI_State_BusyTx,
  SPI_State_BusyTxRx,
} SPI_State;

static SPI_State imu_state = SPI_State_Ready;
static void (*transferCompleteCallback)(void) = (void (*)())NULL;

void spi_Init() {

  // SPI1 clk
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  // SPI1_RX DMA Init
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  DMA1_Stream0->CR = (0 << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC;
  DMA1_Stream0->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  DMAMUX1_Channel0->CCR = DMA_REQUEST_SPI1_RX;
  DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF0;

  // SPI1_TX DMA Init
  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  DMA1_Stream1->CR = (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC;
  DMA1_Stream1->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  DMA1->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
  DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI1_TX;
  DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF1;

  imu_state = SPI_State_Ready;

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

  if (imu_state != SPI_State_Ready) return -1;
  imu_state = SPI_State_BusyTx;

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
          error = -1;
          break;
        }
      }
    }
  }
  tickstart = HAL_GetTick();
  while(!(SPI1->SR & SPI_SR_EOT)) {
    if((HAL_GetTick() - tickstart) >= timeout * size) {
      error = -1;
      break;
    }
  }

  SPI1->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_UDRC;
  SPI1->CR1 &= ~SPI_CR1_SPE;
  imu_state = SPI_State_Ready;
  return error;
}

int spi_TransferDMA(uint8_t *txBuf, uint8_t*rxBuf, int size) {

  if (imu_state != SPI_State_Ready) return -1;
  imu_state = SPI_State_BusyTxRx;

  SPI1->CFG2 &= ~SPI_CFG2_COMM;
  SPI1->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF0;
  DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  DMA1_Stream0->CR &= ~DMA_SxCR_DBM;
  DMA1_Stream0->PAR = (uint32_t)&SPI1->RXDR;
  DMA1_Stream0->M0AR = (uint32_t)rxBuf;
  DMA1_Stream0->NDTR = size;
  DMA1_Stream0->CR = (DMA1_Stream0->CR & ~(DMA_SxCR_DMEIE | DMA_SxCR_HTIE)) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  SPI1->CFG1 |= SPI_CFG1_RXDMAEN;
  DMA1_Stream0->CR |= DMA_SxCR_EN;

  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF1;
  DMA1->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
  DMA1_Stream1->FCR |= DMA_SxFCR_FEIE;
  DMA1_Stream1->CR &= ~DMA_SxCR_DBM;
  DMA1_Stream1->PAR = (uint32_t)&SPI1->TXDR;
  DMA1_Stream1->M0AR = (uint32_t)txBuf;
  DMA1_Stream1->NDTR = size;
  DMA1_Stream1->CR = (DMA1_Stream1->CR & ~(DMA_SxCR_DMEIE | DMA_SxCR_HTIE)) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_TSIZE) | (size << SPI_CR2_TSIZE_Pos);
  SPI1->CFG1 |= SPI_CFG1_TXDMAEN;

  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_CSTART;

  DMA1_Stream1->CR |= DMA_SxCR_EN;

  return 0;
}

void SPI1_IRQHandler(void) {

  if(SPI1->SR & SPI_SR_EOT) {
    SPI1->IFCR |= SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC;
    SPI1->IER &= ~SPI_IER_EOTIE;
    SPI1->IFCR |= SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_TIFREC;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->IER &= ~(SPI_IER_EOTIE | SPI_IER_TXPIE | SPI_IER_RXPIE | SPI_IER_DXPIE | SPI_IER_UDRIE | SPI_IER_OVRIE | \
                              SPI_IER_TIFREIE | SPI_IER_MODFIE);
  SPI1->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);    imu_state = SPI_State_Ready;
    if(transferCompleteCallback) transferCompleteCallback();
  }
}

void DMA1_Stream0_IRQHandler(void) {

  uint32_t isr = DMA1->LISR;
  if(isr & DMA_LISR_TEIF0) {
    if(DMA1_Stream0->CR & DMA_SxCR_TEIE) {
      DMA1_Stream0->CR &= ~DMA_SxCR_TEIE;
      DMA1->LIFCR = DMA_LIFCR_CTEIF0;
    }
  }
  if(isr & DMA_LISR_TCIF0) {
    if(DMA1_Stream0->CR & DMA_SxCR_TCIE) {
      DMA1->LIFCR = DMA_LIFCR_CTCIF0;
      DMA1_Stream0->CR &= ~DMA_SxCR_TCIE;
      SPI1->IER |= SPI_IER_EOTIE; // kick SPI1 EOT
    }
  }
}

void DMA1_Stream1_IRQHandler(void) {

  uint32_t isr = DMA1->LISR;
  if(isr & DMA_LISR_FEIF1) {
    if(DMA1_Stream1->FCR & DMA_SxFCR_FEIE) {
      DMA1->LIFCR = DMA_LIFCR_CFEIF1;
    }
  }
  if(isr & DMA_LISR_TEIF1) {
    if(DMA1_Stream1->CR & DMA_SxCR_TEIE) {
      DMA1_Stream1->CR &= ~DMA_SxCR_TEIE;
      DMA1->LIFCR = DMA_LIFCR_CTEIF1;
    }
  }
  if(isr & DMA_LISR_TCIF1) {
    if(DMA1_Stream1->CR & DMA_SxCR_TCIE) {
      DMA1->LIFCR = DMA_LIFCR_CTCIF1;
      DMA1_Stream1->CR &= ~DMA_SxCR_TCIE;
    }
  }
}
