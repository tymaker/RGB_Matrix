#ifndef __stm32LED_H
#define __stm32LED_H

#include "stm32f10x.h"
#include "SysTick.h"

#define OE_H     GPIO_SetBits(GPIOC, GPIO_Pin_7)	   // OE
#define OE_L     GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define SCK_H    GPIO_SetBits(GPIOC, GPIO_Pin_6)     // SCK
#define SCK_L    GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define LE_H     GPIO_SetBits(GPIOB, GPIO_Pin_13)     // LE
#define LE_L     GPIO_ResetBits(GPIOB, GPIO_Pin_13)   

#define A_H      GPIO_SetBits(GPIOC, GPIO_Pin_4)     // A
#define A_L      GPIO_ResetBits(GPIOC, GPIO_Pin_4)   
#define B_H      GPIO_SetBits(GPIOB, GPIO_Pin_10)    // B
#define B_L      GPIO_ResetBits(GPIOB, GPIO_Pin_10)   
#define C_H      GPIO_SetBits(GPIOC, GPIO_Pin_5)     // C
#define C_L      GPIO_ResetBits(GPIOC, GPIO_Pin_5)   

#define R1_H     GPIO_SetBits(GPIOC, GPIO_Pin_0)     // R1
#define R1_L     GPIO_ResetBits(GPIOC, GPIO_Pin_0)   
#define R2_H     GPIO_SetBits(GPIOC, GPIO_Pin_2)     // R2
#define R2_L     GPIO_ResetBits(GPIOC, GPIO_Pin_2)   
#define G1_H     GPIO_SetBits(GPIOB, GPIO_Pin_11)     // G1
#define G1_L     GPIO_ResetBits(GPIOB, GPIO_Pin_11)   
#define G2_H     GPIO_SetBits(GPIOB, GPIO_Pin_12)     // G2
#define G2_L     GPIO_ResetBits(GPIOB, GPIO_Pin_12)   
#define B1_H     GPIO_SetBits(GPIOC, GPIO_Pin_1)     // B1
#define B1_L     GPIO_ResetBits(GPIOC, GPIO_Pin_1)   
#define B2_H     GPIO_SetBits(GPIOC, GPIO_Pin_3)     // B2
#define B2_L     GPIO_ResetBits(GPIOC, GPIO_Pin_3)   

#define H_H     GPIO_SetBits(GPIOA, GPIO_Pin_12)     // B2
#define H_L     GPIO_ResetBits(GPIOA, GPIO_Pin_12)   
#define LED_GREEN_ON   GPIO_ResetBits(GPIOC, GPIO_Pin_8);  //LED_GREEN_ON
#define LED_GREEN_OFF  GPIO_SetBits(GPIOC, GPIO_Pin_8);    //LED_GREEN
#define LED_BLUE_ON    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
#define LED_BLUE_OFF   GPIO_SetBits(GPIOC, GPIO_Pin_9);
#define LED_RED_ON     GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#define LED_RED_OFF    GPIO_SetBits(GPIOC, GPIO_Pin_12);

#define MAX_light 64

#define countof(a)      (sizeof(a) / sizeof(*(a)))

void RGB_Write_Byte(u32 RED,u32 GREEN,u32 BLUE,u8 Page);
void RGB_Clear(u8 Page);
void RGB_Write(unsigned char Line);  //Ð´ÈëÊý¾Ý
void display(void);
void Display_num(u16 Num1,u16 Num2,u16 Num3,u16 Num4);
void display_HZ(void);


#endif /* __SYSTICK_H */
