/******************** (C) COPYRIGHT 2012  Team **************************
 * �ļ���  ��Boost_pwm.c
 * ����    ��         
 * ʵ��ƽ̨��STM32F103C8T6
 * Ӳ�����ӣ�---------------------
 *          |  PA.08: (TIM1_CH1)  -- BOOST PWM  |
 *          |  PA.07: (TIM3_CH2)  |
 *      	  |  PB.00: (TIM3_CH3)  | 
 *    		  |  PB.01: (TIM3_CH4)  |
 *           ---------------------    			
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "Boost_pwm.h"

volatile u16 BoostPwm_Duty = 0;       /* PWM�źŵ�ƽ����ֵ,��ֵ������PWM��ռ�ձ� */  
/*
 * ��������TIM1_GPIO_Config
 * ����  ������TIM1�������PWMʱ�õ���I/O
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	//PCLK1����2��Ƶ����ΪTIM1��ʱ��Դ����72MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

  /* GPIOA  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  /*GPIOA Configuration: TIM1 channel 1  alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  /* GPIOA  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

  /*GPIOA Configuration: TIM1 channel 2N  alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

/*
 * ��������TIM1_Mode_Config
 * ����  ������TIM1�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
	
/* -----------------------------------------------------------------------
    TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIM1CLK = 72 MHz, Prescaler = 0x0, TIM3 counter clock = 72 MHz
    TIM1 ARR Register = 999 => TIM1 Frequency = TIM3 counter clock/(ARR + 1)
    TIM1 Frequency = 72 KHz.
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM3_ARR)* 100 = 50%
  ----------------------------------------------------------------------- */

  /* Time base configuration */
  TIM_DeInit(TIM1);	
  TIM_TimeBaseStructure.TIM_Period = 999;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
	TIM_OCStructInit(&TIM_OCInitStructure);                    /*��λ����һ����ü���*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = BoostPwm_Duty;	                 //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 

//  /* PWM1 Mode configuration: Channel2,TEST LED */  
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = BoostPwm_Duty;
//  //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//������ͨ��
// // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	  

//  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);			 // ʹ��TIM1���ؼĴ���ARR
  TIM_CtrlPWMOutputs(TIM1, ENABLE);        /*��һ������ֻ���timer1��timer8*/ 
  TIM_Cmd(TIM1, DISABLE);                   //ʹ�ܶ�ʱ��1	
	//TIM_Cmd(TIM1, ENABLE);                   //ʹ�ܶ�ʱ��1	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/*
 * ��������TIM1_PWM_Init
 * ����  ��TIM1 ���PWM�źų�ʼ����ֻҪ�����������
 *         TIM1��1ͨ���ͻ���PWM�ź����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void Boost_PWM_Init(void)
{
	TIM1_GPIO_Config();
	TIM1_Mode_Config();	
}


void Start_Boot_PWM(void)
{ 
	TIM_Cmd(TIM1, ENABLE);
  
	BoostPwm_Duty = 50;
  TIM_SetCompare1( TIM1,BoostPwm_Duty);
  
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);	
	
}

void Stop_Boot_PWM(void)
{
  TIM_Cmd(TIM1, DISABLE);	
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);

}


void TIM1_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//�����ж�
	{
    TIM_SetCompare1( TIM1,BoostPwm_Duty);//ʹ��Ƶ��1����
	}
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

}

/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
