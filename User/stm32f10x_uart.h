#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>
#define Max_UART_String 150
#define TRUE 1
#define FALSE 0

void USART1_Config(void);
char strcmp1(char *,char *);
int fputc(int ch, FILE *f);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);

#endif /* __USART1_H */
