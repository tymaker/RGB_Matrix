#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"
void SysTick_Init(void);
void Delay_us(__IO u32 nTime);
void TimingDelay_Decrement(void);
#endif /* __SYSTICK_H */
