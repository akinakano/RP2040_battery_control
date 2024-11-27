#ifndef __TIMER_H__
#define __TIMER_H__

#define TIMER_CLOCK_SOURCE_FREQ (240000000)
#define MOTION_CONTROL_FREQ     (1600)
#define HZ_FREQ                 (100)
#define EXIMU_HEATER_PWM_FREQ   (1000)

void Timer_Init();

#endif // __TIMER_H__
