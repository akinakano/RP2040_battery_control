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

    // // Battery SMBUS interval priority 33
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 33);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}
