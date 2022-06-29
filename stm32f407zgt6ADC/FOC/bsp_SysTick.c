/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
  *          常用的有 1us 10us 1ms 中断。     
  ******************************************************************************
  */
  
#include "bsp_SysTick.h"

static __IO u32 TimingDelay;
 
/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (HAL_SYSTICK_Config(SystemCoreClock / 1000000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

//void SysTick_Init_ms(void)
//{
//	/* SystemFrequency / 1000    1ms中断一次
//	 * SystemFrequency / 100000	 10us中断一次
//	 * SystemFrequency / 1000000 1us中断一次
//	 */
//	if (HAL_SYSTICK_Config(SystemCoreClock / 1000))
//	{ 
//		/* Capture error */ 
//		while (1);
//	}
//}



///**
//  * @brief   us延时程序,1us为一个单位
//  * @param  
//  *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 1us = 1us
//  * @retval  无
//  */
//void delay_us(__IO u32 nTime)
//{ 
//	TimingDelay = nTime;	

//	while(TimingDelay != 0);
//}

///**
//  * @brief  获取节拍程序
//  * @param  无
//  * @retval 无
//  * @attention  在 SysTick 中断函数 SysTick_Handler()调用
//  */
//void TimingDelay_Decrement(void)
//{
//	if (TimingDelay != 0x00)
//	{ 
//		TimingDelay--;
//	}
//}

//void Delay(__IO uint32_t nCount)
//{
//  for(; nCount != 0; nCount--);
//}



/*********************************************END OF FILE**********************/
