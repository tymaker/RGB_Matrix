/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V2.0
  * @date    2013-xx-xx
  * @brief   STM32 RGB点阵显示程序
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "Systick.h"
#include "stm32_LED.H"
#include "stm32f10x_uart.h"
#include "dht11.h"
#include "spi_flash.h"
#include "Init.h"
#include "rtc.h"
#include "i2c_ee.h"

/****************************  SPI  **************/
/* 获取缓冲区的长度 */
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define  BufferSize (countof(Tx_Buffer)-1)  //获取数组大小
#define  Read_BufferSize 128

#define  FLASH_WriteAddress     0x0000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress
#define  sFLASH_ID              0xEF3015
#define  SPI_FLASH_NUM          3             //字数  要÷4
/**************** I2C ****************************/
#define EEP_Firstpage      0x00   //i2c 地址
#define EEP_NUMBER 2              //
u8 I2c_Buf_Write[EEP_NUMBER];     //写缓冲
u8 I2c_Buf_Read[EEP_NUMBER];      //读缓冲
/*************************************************/
#define System_Error   0xa3;      //系统错误
/***********系统计时器**********/
extern __IO u32 Timing;           //系统systick定时器变量
/***********显示缓存************/
extern u8 String_DisCache_R[];    //显示缓存
extern u8 String_DisCache_G[];
extern u8 String_DisCache_B[];
extern u8 String_number[];
extern __IO u8 hour,minute,second;//小时，分钟，秒，全局变量

extern char g_DatRev[Max_UART_String]; //串口缓冲变量
extern u8 g_DatRev_num;           //串口计数变量
extern u8 UART_FLAG;              //串口标志位
//extern STR_num;
u32 String_num=0;                 //显示读取地址变量
DHT11_Data_TypeDef DHT11_Data;    //温度湿度缓冲
__IO u32 time;                    // ms 更新速度计时变量
__IO u8 Display_flag=0x01;        //显示模式标志 模式变量

uint8_t Tx_Buffer[] = {
0

};

void I2C_Test(void)
{
	printf("写入的数据\n\r");
  I2c_Buf_Write[0] = 0x01;      //显示模式变量
  I2c_Buf_Write[1] = 0x03;      //字符个数计数器
  //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
	I2C_EE_BufferWrite( I2c_Buf_Write, EEP_Firstpage, EEP_NUMBER);	 
  
  printf("\n\r读出的数据\n\r");
  //将EEPROM读出数据顺序保持到I2c_Buf_Read中 
	I2C_EE_BufferRead(I2c_Buf_Read, EEP_Firstpage, EEP_NUMBER); 

  //将I2c_Buf_Read中的数据通过串口打印
}

int main(void)
{
	  //SystemInit();
    SysTick_Init();
    /* USART1 config 115200 8-N-1 */
	  USART1_Config();
	  printf("\n\n********This is Maxire Display System**********");
		/* 2M串行flash W25X16初始化 */
		SPI_FLASH_Init();
	  EXTI_Config();
	  TIM2_Configuration();
		/*初始化DTT11的引脚*/
		DHT11_GPIO_Config();
	  /* I2C 外设初(AT24C02)始化 */
  	I2C_EE_Init();
	  NVIC_Configuration();
	  /* led CONFIG  */
		LED_Init_io();
    Set_time();

		Time_Show();
		/*调用Read_DHT11读取温湿度，若成功则输出该信息*/
//	  Delay_us(1000);
		if( Read_DHT11(&DHT11_Data)==SUCCESS)
		{
			printf("[OK] 湿度为%d.%d ％RH ，温度为 %d.%d℃ \r\n",\
			DHT11_Data.humi_int,DHT11_Data.humi_deci,DHT11_Data.temp_int,DHT11_Data.temp_deci);
			//printf("\r\n 湿度:%d,温度:%d \r\n" ,DHT11_Data.humi_int,DHT11_Data.temp_int);
		}
		else
		{
			printf("[ERROR] Read DHT11 ERROR!\r\n");
		}
//    I2C_Test();
		
/*********************** RTC  *****************/
	  while(1)   //循环
		{
        if(UART_FLAG==0x01)   //如果标志位第一位置一说明发生接收到'\n'
				{
						UART_FLAG &= 0xFE;
						printf("String is:%s\n",g_DatRev);
					  //当接受到数据后，进入这个函数，分析处理
					  
					  if(strcmp1(g_DatRev,"AT+RST\r\n"))
						{
							printf("OK");
            }

						if(strcmp1(g_DatRev,"AT+SPI_FLASH_BulkErase\r\n"))
						{
								SPI_FLASH_BulkErase();
                printf("[OK] SPI_FLASH_BulkErase OK");
            }
						
						if(strcmp1(g_DatRev,"AT+A"))
						{
                /* 将发送缓冲区的数据写到flash中 */
//								SPI_FLASH_BulkErase();
                printf("[OK] SPI_FLASH_BulkErase OK");
//                SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, BufferSize);
							  printf("OK");
            }
						while(g_DatRev_num)  //清空接受缓冲器
						{
                 g_DatRev[g_DatRev_num]=0;
							   g_DatRev_num--;
						}
        }
				else if(UART_FLAG==0x02)  //如果发生了溢出 
				{
            UART_FLAG &= 0xFD;  //清除标志位
						printf("\r\n Data overflow \r\n");
						while(g_DatRev_num)    //清空接受缓冲
						{
                 g_DatRev[g_DatRev_num]=0;
							   g_DatRev_num--;
						}
        }
				
				if(Display_flag==0)  //判断是否为1 为1则显示时间
				{
					  if(time>=1000)
						{
                if( Read_DHT11(&DHT11_Data)==SUCCESS){}
						    Show_Time(RTC_GetCounter());
							  time=0;
            }
						Display_num(DHT11_Data.humi_int,DHT11_Data.temp_int,hour,minute);
						//Delay_us(25);
				}
				else         //否则不为1 则显示汉字
				{
					  //if(time>100)
						display_HZ();
						if ( time >= 1000) /* 1s 时间到 */
						{
							time = 0;
							LED_BLUE_OFF;
							/* LED1 关闭 */
							LED_GREEN_ON;
							
							if((Display_flag&0x01)==0x01)
							    SPI_FLASH_BufferRead(String_DisCache_R, (String_num*128), Read_BufferSize);  //获取flash中数据
							if((Display_flag&0x02)==0x02)
							    SPI_FLASH_BufferRead(String_DisCache_G, (String_num*128), Read_BufferSize);  //获取flash中数据
							if((Display_flag&0x04)==0x04)
							    SPI_FLASH_BufferRead(String_DisCache_B, (String_num*128), Read_BufferSize);  //获取flash中数据

							String_num=String_num+1;
							if(String_num>=SPI_FLASH_NUM)       //判断如果大于等于spi flash num 总字数除以4
									String_num=FLASH_WriteAddress;  //
						}
//					  Delay_us(1);
				}
    }
}

/*********************************************END OF FILE**********************/

