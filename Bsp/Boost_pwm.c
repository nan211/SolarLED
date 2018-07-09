/******************** (C) COPYRIGHT 2012  Team **************************
 * 文件名  ：Boost_pwm.c
 * 描述    ：         
 * 实验平台：STM32F103C8T6
 * 硬件连接：---------------------
 *          |  PA.08: (TIM1_CH1)  -- BOOST PWM  |
 *          |  PA.07: (TIM3_CH2)  |
 *      	  |  PB.00: (TIM3_CH3)  | 
 *    		  |  PB.01: (TIM3_CH4)  |
 *           ---------------------    			
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "Boost_pwm.h"

volatile u16 BoostPwm_Duty = 0;       /* PWM信号电平跳变值,该值决定了PWM的占空比 */  
/*
 * 函数名：TIM1_GPIO_Config
 * 描述  ：配置TIM1复用输出PWM时用到的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	//PCLK1经过2倍频后作为TIM1的时钟源等于72MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

  /* GPIOA  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  /*GPIOA Configuration: TIM1 channel 1  alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  /* GPIOA  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

  /*GPIOA Configuration: TIM1 channel 2N  alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

/*
 * 函数名：TIM1_Mode_Config
 * 描述  ：配置TIM1输出的PWM信号的模式，如周期、极性、占空比
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
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
  TIM_TimeBaseStructure.TIM_Period = 999;       //当定时器从0计数到999，即为1000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	    //设置预分频：不预分频，即为72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
	TIM_OCStructInit(&TIM_OCInitStructure);                    /*复位，这一步最好加上*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = BoostPwm_Duty;	                 //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 

//  /* PWM1 Mode configuration: Channel2,TEST LED */  
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = BoostPwm_Duty;
//  //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//开反向通道
// // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	  

//  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);			 // 使能TIM1重载寄存器ARR
  TIM_CtrlPWMOutputs(TIM1, ENABLE);        /*这一个函数只针对timer1和timer8*/ 
  TIM_Cmd(TIM1, DISABLE);                   //使能定时器1	
	//TIM_Cmd(TIM1, ENABLE);                   //使能定时器1	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/*
 * 函数名：TIM1_PWM_Init
 * 描述  ：TIM1 输出PWM信号初始化，只要调用这个函数
 *         TIM1的1通道就会有PWM信号输出
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
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
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//更新中断
	{
    TIM_SetCompare1( TIM1,BoostPwm_Duty);//使能频道1配置
	}
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

}

/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
