/******************** (C) COPYRIGHT 2012  Team ***************************
 * 文件名  ：SysTick.c
 * 描述    ：SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
 *           常用的有 1us 10us 1ms 中断。         
 * 实验平台：STM32F103C8T6
 * 硬件连接：-----------------
 *          |                 |
 *          |      无         |
 *          |                 |
 *           -----------------
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "SysTick.h"
#include "stdbool.h"

__IO u32 TimingDelay;
__IO u32 ModeDelay;

extern uint32_t PIR_DetectDelay_CNT;

/*
* 函数名：SysTick_Init
* 描述  ：启动系统滴答定时器 SysTick
* SystemFrequency / 1        1000ms中断一次
* SystemFrequency / 10	     100ms中断一次
* SystemFrequency / 100      10ms中断一次
* SystemFrequency / 1000     1ms中断一次
* SystemFrequency / 100000	 10us中断一次
* SystemFrequency / 1000000   1us中断一次
*
* 输入  ：无
* 输出  ：无
* 调用  ：外部调用 
*/
void SysTick_Init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0库版本,1ms中断一次
	{ 
		/* Capture error */ 
		while (1);
	}
		// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;
}


/*
 * 函数名：Delay_ms
 * 描述  ：ms延时程序,1ms为一个单位
 * 输入  ：- nTime
 * 输出  ：无
 * 调用  ：Delay_ms( 1 ) 则实现的延时为 1 ms
 *       ：外部调用 
 */

void Delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	while(TimingDelay != 0);
}


/*
 * 函数名：TimingDelay_Decrement
 * 描述  ：获取节拍程序
 * 输入  ：无
 * 输出  ：无
 * 调用  ：在 SysTick 中断函数 SysTick_Handler()调用
 */  
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
	  TimingDelay--;
	}	
}



void PIRDelay_Decrement(void)
{
	if (PIR_DetectDelay_CNT != 0x00)
	{ 
	  PIR_DetectDelay_CNT--;
	}	
}

void ModeDelay_Decrement(void)
{
	if (ModeDelay != 0x00)
	{ 
	  ModeDelay--;
	}	
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
  PIRDelay_Decrement();
	ModeDelay_Decrement();
}


/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
