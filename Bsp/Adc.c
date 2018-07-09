/******************** (C) COPYRIGHT 2018  Team ***************************
 * �ļ���  ��adc.c
 * ����    ��adcӦ�ú�����        
 * ʵ��ƽ̨��STM32F103C8T6
 * Ӳ�����ӣ�----------------------------------------------------
 *          |                                                        |
 *          | 1-PA0 - ADC1.0 ����NTC���������ѹ�㣬���ڲ����ⲿ�¶� |
 *          | 2-PA1 - ADC1.1 ����LED����������·�����ڲ������ص���   |
 *          | 3-PA2 - ADC1.2 ����Boost��ѹ�����ص�������·         |
 *          | 4-PA3 - ADC1.3 ����PV̫���ܰ����·��������·      |
 *          | 5-PA4 - ADC1.4 ����PV̫���ܰ������ѹ�����̫���ܰ��ѹ|
 *          | 6-PA5 - ADC1.5 ����﮵�ط�ѹ��·�����﮵�ص�ѹ      |
 *          | 7-Temp- ADC1.16 STM32Ƭ���¶�                          |
 *          | 8-Temp- ADC1.17 STM32Ƭ�ϲο���ѹ                      |
 *          |                                                        |
 *           ----------------------------------------------------
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "adc.h"
#include "misc.h"
#include "Usart1.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)   //ADC1  DMA���͵�ַ

__IO uint16_t ADC_ConvertedValue[20][8];
__IO uint16_t ADC_FilterValue[8];


volatile uint8_t ADC_DMA_Finish;


/*******************************************************************
*ADC1_GPIO_Config()
*�������ܣ���ʼ��ADC GPIO
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
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
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// PA01234,����ʱ������������
	/* Configure PB.1  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);				// PB1,����ʱ������������
}


/*******************************************************************
*ADC1_Mode_Config()
*�������ܣ�����ADC1�Ĺ���ģʽΪMDAģʽ
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	       //ADC���ݼĴ�����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;   //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 160;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           //�ڴ��ַ�Լ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                 //ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //ʹ��DMA��������ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* ADC1 configuration */
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	    //����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	        //����ɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	    //��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	            //�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 	                          //Ҫת����ͨ����Ŀ8
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	ADC_TempSensorVrefintCmd(ENABLE);    //ʹ���ڲ��¶ȴ�����

	
	/*����ADC1��ͨ��NΪ55.	5���������ڣ�����ΪN */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);    //�ⲿ�¶�
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);    //���ص���
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);    //Boost�ܵ���
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);    //̫���ܰ��������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);    //̫���ܰ������ѹ
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_55Cycles5);    //��ص�ѹ

	ADC_RegularChannelConfig(ADC1,ADC_Channel_TempSensor,7,ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1,ADC_Channel_Vrefint,8,ADC_SampleTime_55Cycles5); 
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
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
      
//  ADCת�������Ժ󣬶�ȡADC_DR�Ĵ����еĽ����ת���¶�ֵ���㹫ʽ���£�    
//          V25 - VSENSE    
//  T(��) = ------------  + 25    
//           Avg_Slope    
//  V25��  �¶ȴ�������25��ʱ �������ѹ������ֵ1.43 V��    
//  VSENSE���¶ȴ������ĵ�ǰ�����ѹ����ADC_DR �Ĵ����еĽ��ADC_ConvertedValue֮���ת����ϵΪ��    
//            ADC_ConvertedValue * Vdd    
//  VSENSE = --------------------------    
//            Vdd_convert_value(0xFFF)    
//  Avg_Slope���¶ȴ����������ѹ���¶ȵĹ�������������ֵ4.3 mV/�档    
   
	if(ADC_DMA_Finish)
	{
	  ADC_ValueAVGFilter();
		ADC_DMA_Finish = 0x00;
	}	
	
  Vtemp_sensor = ADC_FilterValue[6] * 330 / 4096;   
  Current_Temp = (s32)(143 - Vtemp_sensor) * 10000 / 43 + 2500; 

	#ifdef  ADC_DEBUG_INFO
		printf("GetSTM32Temp:%d��\r\n",Current_Temp);
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
	
	temp = ADC_FilterValue[4] * 33.0f  / 4096;     //90K|10K �����ѹ,Vadc = Vpv * 10/100 ,Vpv =  Vadc * 100 /10 = (3.3 * adc  / 4096) * 10/100 
	
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
	temp = (u16)(ADC_FilterValue[3] * 330 / 4096);  //0.1���������裬�Ŵ�101����
	
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
	temp = (u16)(ADC_FilterValue[2] * 33000 / 4096);  //0.1����������
	
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
	temp = (u16)(ADC_FilterValue[1] * 330 / 4096);  //0.1���������裬�Ŵ�101����
	
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
*�������ܣ���ʼ��ADC
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
	ADC_DMA_NVIC_Configuration();
}


/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/

