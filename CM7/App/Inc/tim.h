#ifndef TIM_H
#define TIM_H

#define TIMER_CLOCK_SOURCE_FREQ (240000000)
#define MOTION_CONTROL_FREQ     (1600)
#define COMM_APMP_RX_IRQ_FREQ   (1600)
#define COMM_APMP_TX_IRQ_FREQ   (50)

#define Comm_APMP_Rx_IrqHandler  TIM2_IRQHandler
#define Comm_APMP_Tx_IrqHandler  TIM5_IRQHandler
#define MotionControl_IrqHandler TIM4_IRQHandler
#define Comm_APMP_Parse_IrqHandler TIM8_BRK_TIM12_IRQHandler
#define Comm_Battery_IrqHandler  TIM8_UP_TIM13_IRQHandler
void TIM2_Init();
void TIM4_Init();
void TIM5_Init();
void TIM12_Init();
void TIM13_Init();

#endif
