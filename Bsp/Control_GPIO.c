/******************** (C) COPYRIGHT 2018 Team **************************
 * 文件名  ：led.c
 * 描述    ：        
 * 实验平台：STM32F103C8T6

 * 硬件连接：-----------------
 *          |   PB13 - LED1    |
 *          |   PB12 - LED2    |
 *          |   PA11 - OverLoad_EN     |
 *           ----------------- 
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "Control_GPIO.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(  LED1_RCC | LED2_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED1_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED1_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED2_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED2_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(LED1_PORT, LED1_PIN);	 // turn off all led
	GPIO_ResetBits(LED2_PORT, LED2_PIN);	 // turn off all led

}

/*
 * 函数名：OverLoad_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void OverLoad_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(  RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_11);	 // Disable OverLoad_EN
}


void Contorl_GPIO_Config(void)
{
  LED_GPIO_Config();
	OverLoad_GPIO_Config();
}




/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
