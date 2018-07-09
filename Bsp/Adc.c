/******************** (C) COPYRIGHT 2018  Team ***************************
 * 文件名  ：adc.c
 * 描述    ：adc应用函数库        
 * 实验平台：STM32F103C8T6
 * 硬件连接：----------------------------------------------------
 *          |                                                        |
 *          | 1-PA0 - ADC1.0 连接NTC热敏电阻分压点，用于测量外部温度 |
 *          | 2-PA1 - ADC1.1 连接LED电流采样电路，用于测量负载电流   |
 *          | 3-PA2 - ADC1.2 连接Boost升压主开关电流检测电路         |
 *          | 4-PA3 - ADC1.3 连接PV太阳能板充电回路电流检测电路      |
 *          | 5-PA4 - ADC1.4 连接PV太阳能板输入分压，检测太阳能板电压|
 *          | 6-PA5 - ADC1.5 连接锂电池分压电路，检测锂电池电压      |
 *          | 7-Temp- ADC1.16 STM32片上温度                          |
 *          | 8-Temp- ADC1.17 STM32片上参考电压                      |
 *          |                                                        |
 *           ----------------------------------------------------
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "adc.h"
#include "misc.h"
#include "Usart1.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)   //ADC1  DMA传送地址

__IO uint16_t ADC_ConvertedValue[20][8];
__IO uint16_t ADC_FilterValue[8];


volatile uint8_t ADC_DMA_Finish;


/*******************************************************************
*ADC1_GPIO_Config()
*函数功能：初始化ADC GPIO
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Configure PA.0/1/2/3/4  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3 |GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// PA01234,输入时不用设置速率
	/* Configure PB.1  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);				// PB1,输入时不用设置速率
}


/*******************************************************************
*ADC1_Mode_Config()
*函数功能：配置ADC1的工作模式为MDA模式
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	       //ADC数据寄存器地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;   //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 160;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           //内存地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                 //循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //使能DMA传输完成中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* ADC1 configuration */
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	    //独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	        //开启扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	    //开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	            //采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 	                          //要转换的通道数目8
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	ADC_TempSensorVrefintCmd(ENABLE);    //使能内部温度传感器

	
	/*配置ADC1的通道N为55.	5个采样周期，序列为N */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);    //外部温度
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);    //负载电流
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);    //Boost管电流
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);    //太阳能板输出电流
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);    //太阳能板输入电压
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_55Cycles5);    //电池电压

	ADC_RegularChannelConfig(ADC1,ADC_Channel_TempSensor,7,ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1,ADC_Channel_Vrefint,8,ADC_SampleTime_55Cycles5); 
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_ValueAVGFilter(void)
{
   int  sum = 0;
   u8 count,i;
	
   for(i=0; i<8; i++)
   {
    for ( count =0;count < ADC_CNV_NUM; count++)    
				{
       sum += ADC_ConvertedValue[count][i];
    }
    ADC_FilterValue[i] = sum / ADC_CNV_NUM;
    sum = 0;
   }
   
}

void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
  {
    ADC_DMA_Finish = 0x01;
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
}


u16 GetSTM32Temp(void)   
{   
    u32 Vtemp_sensor;   
    s32 Current_Temp;   
      
//  ADC转换结束以后，读取ADC_DR寄存器中的结果，转换温度值计算公式如下：    
//          V25 - VSENSE    
//  T(℃) = ------------  + 25    
//           Avg_Slope    
//  V25：  温度传感器在25℃时 的输出电压，典型值1.43 V。    
//  VSENSE：温度传感器的当前输出电压，与ADC_DR 寄存器中的结果ADC_ConvertedValue之间的转换关系为：    
//            ADC_ConvertedValue * Vdd    
//  VSENSE = --------------------------    
//            Vdd_convert_value(0xFFF)    
//  Avg_Slope：温度传感器输出电压和温度的关联参数，典型值4.3 mV/℃。    
   
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	
  Vtemp_sensor = ADC_FilterValue[6] * 330 / 4096;   
  Current_Temp = (s32)(143 - Vtemp_sensor) * 10000 / 43 + 2500; 

	#ifdef  ADC_DEBUG_INFO
		printf("GetSTM32Temp:%d℃\r\n",Current_Temp);
	#endif
  
    return (s16)Current_Temp;   
}  

    
u16 GetSTM32Vref(void)   
{   
  u16 temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}
	
	temp = (u16)(ADC_FilterValue[7] * 330 / 4096);
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetSTM32Vref:%.02fV\r\n",temp/100.0f);
	#endif
  
	return temp;   
} 

float GetBaterryVolt(void)   
{   
  float temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	temp = ADC_FilterValue[5] * 33.0f / 4096;
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetBaterryVolt:%.02fV\r\n",temp);
	#endif
  
	return temp;   
}

float GetSolarVolt(void)   
{   
  float temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	
	temp = ADC_FilterValue[4] * 33.0f  / 4096;     //90K|10K 电阻分压,Vadc = Vpv * 10/100 ,Vpv =  Vadc * 100 /10 = (3.3 * adc  / 4096) * 10/100 
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetSolarVolt:%.02fV\r\n",temp);
	#endif
  
	return temp;   
}

u16 GetSolarCurrent(void)   
{   
  u16 temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	temp = (u16)(ADC_FilterValue[3] * 330 / 4096);  //0.1Ω采样电阻，放大101倍，
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetSolarCurrent::%dmA\r\n",temp);
	#endif
  
	return temp;   
}

u16 GetBoostCurrent(void)   
{   
  u16 temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	temp = (u16)(ADC_FilterValue[2] * 33000 / 4096);  //0.1Ω采样电阻
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetBoostCurrent:%dmA\r\n",temp);
	#endif
  
	return temp;   
}

u16 GetLoadCurrent(void)   
{   
  u16 temp;
	
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	temp = (u16)(ADC_FilterValue[1] * 330 / 4096);  //0.1Ω采样电阻，放大101倍，
	
	#ifdef  ADC_DEBUG_INFO
		printf("GetLoadCurrent:%d mA\r\n",temp);
	#endif
  
	return temp;   
}


void ADC_DMA_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the DMA Interrupt */	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);                  // Enable the DMA Interrupt
}



/*******************************************************************
*ADC1_Init()
*函数功能：初始化ADC
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
	ADC_DMA_NVIC_Configuration();
}


/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/

