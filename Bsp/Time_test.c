/******************** (C) COPYRIGHT 2012  Team ***************************
 * �ļ���  ��Time_test.c
 * ����    ��TIM2 1ms ��ʱӦ�ú�����
 *          
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�----------------------
 *          |					  |
 *          |	        ��     	  |
 *          |					  |
 *           ----------------------
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "Time_test.h"


/*
 * ��������TIM4_NVIC_Configuration
 * ����  ��TIM4�ж����ȼ�����
 * ����  ����
 * ���  ����	
 */
void TIM4_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*TIM_Period--1000   TIM_Prescaler--71 -->�ж�����Ϊ1ms*/
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=1000;		 								/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    		/* �������жϱ�־ */
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4, ENABLE);																		/* ����ʱ�� */
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , DISABLE);		/*�ȹرյȴ�ʹ��*/    
}

void TIM4_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
