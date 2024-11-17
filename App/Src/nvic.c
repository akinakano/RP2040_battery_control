#include    <stm32h747xx.h>
#include    <core_cm7.h>

#include "nvic.h"

// priority : 0(high)~255(low)
void NVIC_Init(void) {

    // comm_MPSV priority 1
    NVIC_SetPriority(UART4_IRQn, 1);
    NVIC_EnableIRQ(UART4_IRQn);

    // motion_controller interval priority  3
    NVIC_SetPriority(TIM4_IRQn, 3);
    NVIC_EnableIRQ(TIM4_IRQn);

    // SPI1 priority 5
    NVIC_SetPriority(DMA1_Stream0_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream1_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 6);
    NVIC_EnableIRQ(SPI1_IRQn);

    // USB-FS priority 8
    NVIC_SetPriority(OTG_FS_IRQn, 8);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    // SMBUS priority 14
    NVIC_SetPriority(I2C1_EV_IRQn, 14);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 14);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    // debug_console priority 16
    NVIC_SetPriority(USART2_IRQn, 16);
    NVIC_EnableIRQ(USART2_IRQn);

    // generic timer 100Hz interval priority 32
    NVIC_SetPriority(TIM3_IRQn, 32);
    NVIC_EnableIRQ(TIM3_IRQn);
}
