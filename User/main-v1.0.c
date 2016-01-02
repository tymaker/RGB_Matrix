/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V2.0
  * @date    2013-xx-xx
  * @brief   STM32 RGB������ʾ����
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#include "stm32f10x.h"
//#include "Systick.h"
#include "stm32_LED.H"




void LED_GPIO_Config(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);   //��ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               //���õ�15��
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //�����������
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //ʱ��50Mhz
	  GPIO_Init(GPIOA,&GPIO_InitStructure);                    //
	  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}

/*
 * ��������NVIC_Configuration
 * ����  ������Ƕ�������жϿ�����NVIC
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* ����P[A|B|C|D|E]0Ϊ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������EXTI_PB0_Config
 * ����  ������ PB0 Ϊ���жϿڣ��������ж����ȼ�
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void EXTI_PB0_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//RCC_APB2Periph_GPIOB | 

	/* config the NVIC(PB0) */
	NVIC_Configuration();

	/* EXTI line gpio config(PB0) */	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // ��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* EXTI line(PB0) mode config */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}

void LED_Init_io()
{
	//LED��ʼ�� led OE 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure1;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  GPIO_InitStructure1.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5  | GPIO_Pin_6 | GPIO_Pin_7;	
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOC, &GPIO_InitStructure1);

/********************************************************************/
	GPIO_SetBits(GPIOC, GPIO_Pin_7);	  // OE��ʼ��Ϊ�ߵ�ƽ ����ʾ
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);  // SCKʱ�ӵ���0
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);  // LE��ʼ��Ϊ�͵�ƽ
  GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // A
	GPIO_ResetBits(GPIOB, GPIO_Pin_10); // B
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);  // C
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);  //R1
  GPIO_ResetBits(GPIOC, GPIO_Pin_2);  //R2
  GPIO_ResetBits(GPIOB, GPIO_Pin_11);  //G1
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);  //G2
  GPIO_ResetBits(GPIOC, GPIO_Pin_1);  //B1
  GPIO_ResetBits(GPIOC, GPIO_Pin_3);  //B2
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);  //
}

/***********           ��ɫ ��ɫ ��ɫ �ֽ�����  ��ɫ            **************/
/*
д��һ���ֽڣ�8���������
*/
void RGB_Write_Byte(u8 RED,u8 GREEN,u8 BLUE,u8 Page)
{
    u8 i;        //��ʼ��
	  u8 r,g,b;    //��ԭɫ����
	  r=RED;                  //��ֵ
	  g=GREEN;
	  b=BLUE;
	  SCK_L;                  //����ʱ��
	  if(Page)  //Page Ϊ1ʱ��д���ϰ벿��
	      for(i=0;i<8;i++)
			  {
					 ((r & 0x01) ==0x01)? R1_H:R1_L;
					 ((g & 0x01) ==0x01)? G1_H:G1_L;
					 ((b & 0x01) ==0x01)? B1_H:B1_L;
					
//            R1=r & 0x01;  //д���ɫ����
//            G1=g & 0x01;  //д����ɫ����
//            B1=b & 0x01;  //д����ɫ����
					  r=r>>1;
					  g=g>>1;
					  b=b>>1;
					  SCK_H;
					  SCK_L;
        }
		else    //д���°벿��
				for(i=0;i<8;i++)
				{
					 ((r & 0x01) ==0x01)? R2_H:R2_L;
					 ((g & 0x01) ==0x01)? G2_H:G2_L;
					 ((b & 0x01) ==0x01)? B2_H:B2_L;
						r=r>>1;
						g=g>>1;
						b=b>>1;
						SCK_H;
						SCK_L;
				}
}
void RGB_Write(unsigned char Line)  //д������
{
	  u8 j;
	  u8 Page;
	  LE_L;       //
	  if(Line<9)
		    Page=1;
		else
			  Page=0;
		OE_H;       //
		switch (Line)
		{
				case 1:A_L;B_L;C_L;break;
				case 5:A_L;B_L;C_H;break;
				case 3:A_L;B_H;C_L;break;
				case 7:A_L;B_H;C_H;break;
				case 2:A_H;B_L;C_L;break;
				case 6:A_H;B_L;C_H;break;
				case 4:A_H;B_H;C_L;break;
				case 8:A_H;B_H;C_H;break;
				case 9:A_L;B_L;C_L;break;
				case 13:A_L;B_L;C_H;break;
				case 11:A_L;B_H;C_L;break;
				case 15:A_L;B_H;C_H;break;
				case 10:A_H;B_L;C_L;break;
				case 14:A_H;B_L;C_H;break;
				case 12:A_H;B_H;C_L;break;
				case 16:A_H;B_H;C_H;break;
				default:OE_H;//A=1;D=1;C=1;
		}
	  #if (MAX_light==32)   //д��һ������
			for(j=0;j<4;j++)
			{
					RGB_Write_Byte(Red_Write[j],Green_Write[j],Blue_Write[j],Page);
			}
	  #else
			for(j=0;j<8;j++)
			{
					RGB_Write_Byte(Red_Write[j],Green_Write[j],Blue_Write[j],Page);
			}
	  #endif
		LE_H;        //����д������
		LE_L;
		OE_L;
	  Delay_us(40);
		OE_H;
}

void display(void)
{
		u8 i;
		for(i=1;i<17;i++)
		{
				Red_Write[0]  = String_R[((i-1)*8)+0];
				Red_Write[1]  = String_R[((i-1)*8)+1];
				Red_Write[2]  = String_R[((i-1)*8)+2];
				Red_Write[3]  = String_R[((i-1)*8)+3];
				Red_Write[4]  = String_R[((i-1)*8)+4];
				Red_Write[5]  = String_R[((i-1)*8)+5];
				Red_Write[6]  = String_R[((i-1)*8)+6];
				Red_Write[7]  = String_R[((i-1)*8)+7];

				Green_Write[0]  = String_G[((i-1)*8)+0];
				Green_Write[1]  = String_G[((i-1)*8)+1];
				Green_Write[2]  = String_G[((i-1)*8)+2];
				Green_Write[3]  = String_G[((i-1)*8)+3];
				Green_Write[4]  = String_G[((i-1)*8)+4];
				Green_Write[5]  = String_G[((i-1)*8)+5];
				Green_Write[6]  = String_G[((i-1)*8)+6];
				Green_Write[7]  = String_G[((i-1)*8)+7];

				Blue_Write[0]  = String_B[((i-1)*8)+0];
				Blue_Write[1]  = String_B[((i-1)*8)+1];
				Blue_Write[2]  = String_B[((i-1)*8)+2];
				Blue_Write[3]  = String_B[((i-1)*8)+3];
				Blue_Write[4]  = String_B[((i-1)*8)+4];
				Blue_Write[5]  = String_B[((i-1)*8)+5];
				Blue_Write[6]  = String_B[((i-1)*8)+6];
				Blue_Write[7]  = String_B[((i-1)*8)+7];
				RGB_Write(i);
		}
		H_H;
		H_L;

}

int main(void)
{
	//SystemInit();
    SysTick_Init();
	  LED_GPIO_Config();
		LED_Init_io();
	  OE_L;   //��ʾ
	  EXTI_PB0_Config();
	  while(1)
		{
//        GPIO_ResetBits(GPIOB,GPIO_Pin_15);
//			  Delay_us(1000000);
//			  GPIO_SetBits(GPIOB,GPIO_Pin_15);
//			  Delay_us(1000000);
			display();
    }
}

/*********************************************END OF FILE**********************/
