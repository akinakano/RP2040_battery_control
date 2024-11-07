#include <stdint.h>
#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"
#include "tim.h"

// tim2の初期化、更新割込みを設定
// 運動制御
void TIM2_Init(void)
{
    //uint32_t u32;

    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;

    // TIM2で周期割込みを設定する
    TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;       // カウント有効化、自動再ロード有効化
    TIM2->PSC = 1 - 1;                             // プリスケーラ 170MHzを1分周
    // 1kHzに設定
    //for debug
    TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE| TIM_CCMR1_OC1FE; //PWMmode1 CH1
    TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE| TIM_CCMR1_OC2FE; //PWMmode1 CH2
    TIM2->CCMR2 = (TIM2->CCMR1 & ~TIM_CCMR2_OC3M_Msk) | (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE| TIM_CCMR2_OC3FE; //PWMmode1 CH3

    TIM2->CCER |= TIM_CCER_CC1E; //PWM CH1 Output
    //TIM2->CCER |= TIM_CCER_CC2E; //PWM CH2 Output
    //TIM2->CCER |= TIM_CCER_CC3E; //PWM CH2 Output

    TIM2->ARR = TIMER_CLOCK_SOURCE_FREQ / MOTION_CONTROL_FREQ - 1; // カウントリセット設定値 32bitなので大きい値をセット出来る
    TIM2->DIER |= TIM_DIER_UIE;                    // 更新割込み有効化
    TIM2->EGR |= TIM_EGR_UG;                       // カウント初期化

    TIM2->CCR1 = (TIMER_CLOCK_SOURCE_FREQ / MOTION_CONTROL_FREQ - 1)/4;
    //TIM2->CCR2 = (TIMER_CLOCK_SOURCE_FREQ / MOTION_CONTROL_FREQ - 1)/4;
    //TIM2->CCR3 = (TIMER_CLOCK_SOURCE_FREQ / MOTION_CONTROL_FREQ - 1)/4;

    /*
    u32 = GPIOA->OTYPER;
    u32 &= 
    */


}

// tim3の初期化、更新割込みを設定、デバッグ用低優先度割込み
// デバッグプリント
void TIM3_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;

    // TIM3で周期割込みを設定する
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;       // カウント有効化、自動再ロード有効化
    TIM3->PSC = 240 - 1;                           // プリスケーラ 240MHzを240分周　1MHz
    TIM3->ARR = 1000000 / MOTION_CONTROL_FREQ - 1; // カウントリセット設定値　1600Hz
    TIM3->DIER |= TIM_DIER_UIE;                    // 更新割込み有効化
    TIM3->EGR |= TIM_EGR_UG;                       // カウント初期化
}

// tim4の初期化、更新割込みを設定
// apmp間通信 受信 ダメだったら受信割込みで取得するようにする
void TIM4_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;

    // TIM5で周期割込みを設定する
    TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;          // カウント有効化、自動再ロード有効化
    TIM4->PSC = 240 - 1;                              // プリスケーラ 240MHzを240分周
    TIM4->ARR = 1000000 / COMM_APMP_RX_IRQ_FREQ - 1;  //　1600Hz
    TIM4->DIER |= TIM_DIER_UIE;                       // 更新割込み有効化
    TIM4->EGR |= TIM_EGR_UG;                          // カウント初期化
}

// tim5の初期化、更新割込みを設定
// apmp間通信 送信
void TIM5_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;

    // TIM5で周期割込みを設定する
    TIM5->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;          // カウント有効化、自動再ロード有効化
    TIM5->PSC = 1 - 1;                                // プリスケーラ 240MHzを1分周
    // SystemControl: 1kHz
    TIM5->ARR = TIMER_CLOCK_SOURCE_FREQ / COMM_APMP_TX_IRQ_FREQ - 1; // カウントリセット設定値 32bitなので大きい値をセット出来る 50Hz
    TIM5->DIER |= TIM_DIER_UIE;                       // 更新割込み有効化
    TIM5->EGR |= TIM_EGR_UG;                          // カウント初期化
}

// tim12の初期化、更新割込みを設定
// apmp間通信 受信解釈
void TIM12_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;

    // TIM12で周期割込みを設定する
    TIM12->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;          // カウント有効化、自動再ロード有効化
    TIM12->PSC = 240 - 1;                              // プリスケーラ 240MHzを240分周
    TIM12->ARR = 1000000 / MOTION_CONTROL_FREQ - 1;    // カウントリセット設定値 1600Hz
    TIM12->DIER |= TIM_DIER_UIE;                       // 更新割込み有効化
    TIM12->EGR |= TIM_EGR_UG;                          // カウント初期化
}

// tim13の初期化、更新割込みを設定
// バッテリー通信
void TIM13_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM13EN;

    // TIM13で周期割込みを設定する
    TIM13->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;           // カウント有効化、自動再ロード有効化
    TIM13->PSC = 24000 - 1;                             // プリスケーラ 240MHzを24000分周 = 10KHz
    TIM13->ARR = 10 * 1000 / 1 - 1;                    // カウントリセット設定値 1Hz
    TIM13->DIER = TIM_DIER_UIE;                         // 更新割込み有効化
    TIM13->EGR = TIM_EGR_UG;                            // カウント初期化
}
