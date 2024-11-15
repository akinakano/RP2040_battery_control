#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include <stdint.h>

extern int Power15V;

void PowerControl_Init(void);
void PowerControl_Handler();
void Heartbeat_Handler(void);

#define POWER_ON_DELAY 30

#endif // __POWER_CONTROL_H__
