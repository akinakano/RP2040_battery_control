#ifndef TIM_H
#define TIM_H

#define TIMER_CLOCK_SOURCE_FREQ (240000000)
#define MOTION_CONTROL_FREQ     (1600)
#define HZ_FREQ                 (100)

void TIM3_Init();
void TIM4_Init();

#endif
