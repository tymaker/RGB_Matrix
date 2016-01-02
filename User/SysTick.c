/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���  ��SysTick.c
 * ����    ��SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
 *           ���õ��� 1us 10us 1ms �жϡ�         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�-----------------
 *          |                 |
 *          |      ��         |
 *          |                 |
 *           -----------------
 * ��汾  ��ST3.0.0
 *
 * ����    ��fire  QQ: 313303034 
 * ����    ��firestm32.blog.chinaunix.net
**********************************************************************************/

#include "SysTick.h"

static __IO u32 TimingDelay;
__IO u32 Timing;

/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (SysTick_Config(72000000 / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
}


/*
 * ��������Delay_us
 * ����  ��us��ʱ����,1msΪһ����λ
 * ����  ��- nTime
 * ���  ����
 * ����  ��Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1ms
 *       ���ⲿ���� 
 */

void Delay_us(__IO u32 nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/*
 * ��������TimingDelay_Decrement
 * ����  ����ȡ���ĳ���
 * ����  ����
 * ���  ����
 * ����  ���� SysTick �жϺ��� SysTick_Handler()����
 */  
void TimingDelay_Decrement(void)
{
	Timing++;
	if(Timing==0xffffffff)
	{
		Timing=0;
	}
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
/******************* (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� *****END OF FILE****/