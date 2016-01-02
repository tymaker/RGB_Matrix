#ifndef __INIT_H
#define	__INIT_H

#include "stm32f10x.h"
#define LED_GREEN_ON   GPIO_ResetBits(GPIOC, GPIO_Pin_8);  //LED_GREEN_ON
#define LED_GREEN_OFF  GPIO_SetBits(GPIOC, GPIO_Pin_8);    //LED_GREEN
#define LED_BLUE_ON    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
#define LED_BLUE_OFF   GPIO_SetBits(GPIOC, GPIO_Pin_9);
#define LED_RED_ON     GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#define LED_RED_OFF    GPIO_SetBits(GPIOC, GPIO_Pin_12);

void LED_Init_io(void);
void TIM2_Configuration(void);
void EXTI_Config(void);
void NVIC_Configuration(void);
#endif /* __INIT_H */
