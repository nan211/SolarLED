//******************************************************************************              
//name:             Infrared_Send.c             
//introduce:        ���⴫������������      
//author:                                                     
//changetime:       2016.11.16    
//******************************************************************************
#include "Infrared_Send.h"
#include "Usart1.h"
#include "Control_GPIO.h"

/*********************�ⲿ����************************/
//���⿪�صı�־λ
u8 gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
u8 gGUA_Infrared_Sending = 0;  //���ⷢ�ͱ�־
u8 GUA_USER_CODE;

//����38K���巭ת����
volatile unsigned short  gGUA_Infrared_Count;

/*
 * ��������IR_Send_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void IR_Send_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( GUA_INFRARED_SEND_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GUA_INFRARED_SEND_PIN;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GUA_INFRARED_SEND_PORT, &GPIO_InitStructure);

  GPIO_SetBits(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN);	 // turn off all led
}



/*TIM_Period-->12+   TIM_Prescaler--71 -->�ж�����Ϊ13us-38kHz*/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period = 12;		 					          /* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    TIM_TimeBaseStructure.TIM_Prescaler = (72 -1);				        /* ʱ��Ԥ��Ƶ�� 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		    /* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);							            /* �������жϱ�־ */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);											                  /* ����ʱ�� */
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);     //ʹ�ܶ�ʱ��2��ʱ��  
}


//******************************************************************************        
//name:             TIM2_IRQHandler        
//introduce:        ��ʱ��2�жϷ�����,������ı�־λ��ʱ,�ܲ���38KƵ��;�ܼ������жϴ�����     
//parameter:        none       
//return:           none      
//author:                              
//changetime:       2016.11.16                      
//******************************************************************************  
void TIM2_IRQHandler(void)
{
	BitAction bGUA_Infrared_OnOff_Status = GUA_INFRARED_OFF;  
    
  //����ֵ����  
  if(gGUA_Infrared_Count > 0)  
  {  
    gGUA_Infrared_Count--;  
  }  
    
  //�����־���򿪣������38K����  
  if(gGUA_Infrared_Flag == GUA_INFRARED_FLAG_ON)  
  {  
    //��ȡ��ǰ�������״̬  
    bGUA_Infrared_OnOff_Status = (BitAction)(GPIO_ReadOutputDataBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN));  
      
    //�����ǰ�رգ����  
    if(bGUA_Infrared_OnOff_Status == GUA_INFRARED_OFF)  
    {  
      GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_ON);
      LED2(ON);      
    }  
    //�����ǰ�򿪣���ر�  
    else  
    {  
      GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_OFF);
      LED2(OFF);			
    }	
  
  }  
  //�����־���ر�  
  else  
  {  
    //�رպ���  
    GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_OFF);
    LED2(OFF);		
  }  
    	
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
	}
}


//******************************************************************************        
//name:             GUA_Infrared_Send        
//introduce:        ���ⷢ��     
//parameter:        nGUA_Data:Ҫ���͵�����       
//return:           none      
//author:                                 
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Send_NByte(u8 GUA_USER_CODE,u8 DataLengh, u8 *nGUA_Data)
{
	u8 i,j = 0;
	u8 datatemp = 0;
	u8 rcr_temp= 0;
  
  //��ʼ�����⣬Ĭ��Ϊ��
	EXTI->IMR &= ~(EXTI_Line6);                               // ���κ������,�ⲿ�ж�6
	
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);     //ʹ�ܶ�ʱ��2��ʱ��
	
  //9ms��ʼ�źŵ͵�ƽ,683*13us=8.879ms
  gGUA_Infrared_Count = 683;	
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
	while(gGUA_Infrared_Count);
  
  //4.5ms��ʼ�źŸߵ�ƽ��341*13=4.433ms
  gGUA_Infrared_Count = 341;
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
	while(gGUA_Infrared_Count);

  	
	#ifdef  INFRARED_SEND_DEBUG_INFO
    printf("Send Infrared_Data:\r\n");
  #endif
	

	//�����û���
	datatemp = GUA_USER_CODE;
  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif 
  
	for(j = 0; j <  8; j++)
	{
		//ͨ�õ�0.56ms�͵�ƽ,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count); 
	
		//��ֵ0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms�ߵ�ƽ��43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//��ֵ1
		else
		{
			//1.69ms�ߵ�ƽ��128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//ִ�иߵ�ƽ
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count); 

		//������λ
		datatemp <<= 1;
	}
	
	//�����û��뷴��
	datatemp = (u8)~GUA_USER_CODE;
  	  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif
  
	for(j = 0; j <  8; j++)
	{
		//ͨ�õ�0.56ms�͵�ƽ,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count);    
	
		//��ֵ0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms�ߵ�ƽ��43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//��ֵ1
		else
		{
			//1.69ms�ߵ�ƽ��128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//ִ�иߵ�ƽ
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count);

		//������λ
		datatemp <<= 1;
	}
	
  //ѭ������NByte������
  for(i = 0; i < DataLengh ; i++)
  {
    datatemp =  *(nGUA_Data + i);
		rcr_temp = rcr_temp + datatemp;
		
	  #ifdef  INFRARED_SEND_DEBUG_INFO
      printf("0x%02X-",datatemp);
    #endif		
		
		for(j = 0; j <  8; j++)
		{
			//ͨ�õ�0.56ms�͵�ƽ,42*13us=0.546ms
			gGUA_Infrared_Count = 42;
			gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
			while(gGUA_Infrared_Count);
		
			//��ֵ0
			if((datatemp & 0x80) == 0)
			{
				//0.565ms�ߵ�ƽ��43*13us=0.559ms
				gGUA_Infrared_Count = 43;          
			}
			//��ֵ1
			else
			{
				//1.69ms�ߵ�ƽ��128*13us=1.664ms
				gGUA_Infrared_Count = 128;
			}
			
			//ִ�иߵ�ƽ
			gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
			while(gGUA_Infrared_Count); 

			//������λ
			datatemp <<= 1;
		}		
  }
		
	//����CRC
	datatemp = (u8)~rcr_temp + 1;
  	  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif
  
	for(j = 0; j <  8; j++)
	{
		//ͨ�õ�0.56ms�͵�ƽ,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count);    
	
		//��ֵ0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms�ߵ�ƽ��43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//��ֵ1
		else
		{
			//1.69ms�ߵ�ƽ��128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//ִ�иߵ�ƽ
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count);

		//������λ
		datatemp <<= 1;
	}
	
  
  //0.263ms�͵�ƽ��Ϊ����λ��20*13=0.26ms
  gGUA_Infrared_Count = 20;
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
	while(gGUA_Infrared_Count); 
  
  //�رպ���
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		    //�����͵�ʱ�򣬹ص�38K��ʱ����ʱ�� 	
	
	#ifdef  INFRARED_SEND_DEBUG_INFO
    printf("\r\n");
  #endif
	
	EXTI->IMR |= EXTI_Line6;                                      // ���κ������,�ⲿ�ж�6
}

//******************************************************************************        
//name:             GUA_Infrared_Send_Init        
//introduce:        ���ⷢ�ͳ�ʼ��     
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Send_Init(void)
{  
  IR_Send_GPIO_Config();
  TIM2_Configuration();
}

