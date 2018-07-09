/******************** (C) COPYRIGHT 2012  Team ***************************
 * �ļ���  ��SysTick.c
 * ����    ��SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
 *           ���õ��� 1us 10us 1ms �жϡ�         
 * ʵ��ƽ̨��STM32F103C8T6
 * Ӳ�����ӣ�-----------------
 *          |                 |
 *          |      ��         |
 *          |                 |
 *           -----------------
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "SysTick.h"
#include "stdbool.h"

__IO u32 TimingDelay;
__IO u32 ModeDelay;

extern uint32_t PIR_DetectDelay_CNT;

/*
* ��������SysTick_Init
* ����  ������ϵͳ�δ�ʱ�� SysTick
* SystemFrequency / 1        1000ms�ж�һ��
* SystemFrequency / 10	     100ms�ж�һ��
* SystemFrequency / 100      10ms�ж�һ��
* SystemFrequency / 1000     1ms�ж�һ��
* SystemFrequency / 100000	 10us�ж�һ��
* SystemFrequency / 1000000   1us�ж�һ��
*
* ����  ����
* ���  ����
* ����  ���ⲿ���� 
*/
void SysTick_Init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0��汾,1ms�ж�һ��
	{ 
		/* Capture error */ 
		while (1);
	}
		// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;
}


/*
 * ��������Delay_ms
 * ����  ��ms��ʱ����,1msΪһ����λ
 * ����  ��- nTime
 * ���  ����
 * ����  ��Delay_ms( 1 ) ��ʵ�ֵ���ʱΪ 1 ms
 *       ���ⲿ���� 
 */

void Delay_ms(__IO u32 nTime)
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
