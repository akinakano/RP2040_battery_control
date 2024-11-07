#include    <stm32h747xx.h>
#include    <core_cm7.h>

#include "nvic.h"

//割込み優先度と割込みEnableを設定する。0~255で優先度をつけられる。0が最優先。
void NVIC_Init(void)
{
    EXTI->IMR1 |= EXTI_IMR1_IM0; // EXTI0割り込みを有効化

    EXTI->RTSR1 |= EXTI_RTSR1_TR0;
    EXTI->IMR1 |= EXTI_IMR1_IM0;
    EXTI->EMR1 &= (~EXTI_EMR1_EM0);

    // EXTI->RTSR1 |= EXTI_RTSR1_TR5;
    // EXTI->C2IMR1 |= EXTI_IMR1_IM5;
    // EXTI->C2EMR1 &= (~EXTI_EMR1_EM5);

    // EXTI->RTSR1 |= EXTI_RTSR1_TR6;
    // EXTI->C2IMR1 |= EXTI_IMR1_IM6;
    // EXTI->C2EMR1 &= (~EXTI_EMR1_EM6);

    // EXTI->RTSR1 |= EXTI_RTSR1_TR7;
    // EXTI->C2IMR1 |= EXTI_IMR1_IM7;
    // EXTI->C2EMR1 &= (~EXTI_EMR1_EM7);

    // EXTI->RTSR1 |= EXTI_RTSR1_TR8;
    // EXTI->C2IMR1 |= EXTI_IMR1_IM8;
    // EXTI->C2EMR1 &= (~EXTI_EMR1_EM8);

    // メカナム速度制御 
    NVIC_SetPriority(TIM2_IRQn, 3);//優先度3
    NVIC_EnableIRQ(TIM2_IRQn);//割込みON
    
    // デバッグ入出力
    NVIC_SetPriority(TIM3_IRQn, 10);//優先度10
    NVIC_EnableIRQ(TIM3_IRQn);//割込みON

    // APMP通信 Rx
    NVIC_SetPriority(TIM4_IRQn, 4);//優先度4
    NVIC_EnableIRQ(TIM4_IRQn);//割込みON

    // // APMP通信 Rx
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 6);//優先度4
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);//割込みON

    // APMP通信 Tx
    NVIC_SetPriority(TIM5_IRQn, 5);//優先度5
    NVIC_EnableIRQ(TIM5_IRQn);//割込みON

    // MPSV通信
    NVIC_SetPriority(USART2_IRQn, 1);//優先度4
    NVIC_EnableIRQ(USART2_IRQn);//割込みON
    
    // MPSV通信
    NVIC_SetPriority(EXTI0_IRQn, 2);//優先度4
    NVIC_EnableIRQ(EXTI0_IRQn);//割込みON

    // // Battery通信
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 128);//優先度128
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);//割込みON

    // NVIC_SetPriority(EXTI9_5_IRQn, 4);//優先度4
    // NVIC_EnableIRQ(EXTI9_5_IRQn);//割込みON
}
