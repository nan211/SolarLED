//******************************************************************************              
//name:             Infrared_Recv.c             
//introduce:        ���⴫������������      
//author:   
//2080620 : Infrared_Recv.c line271,line277,����2��ָ����ڻָ��������úͻض�����
//changetime:       2016.11.16    
//******************************************************************************
#include "Infrared_Recv.h"
#include "Infrared_Send.h"
#include "Usart1.h"
#include "UserApp.h"

bool Ir_RecvComplete;      //������ɱ�־λ
bool Ir_RecvFlag;          //��ʾ��ʼ����
bool PIR_DetectionFlag;          //��ʾ��ʼ����

uint8_t Ir_RecvBuff[GUA_INFRARED_FRAME_LEN * 8 + 1];   //���ڼ�¼�����½���֮���ʱ��
volatile uint16_t IrCode_idx;        //�����������յ�����ֵ
volatile uint16_t IrTimecnt;

extern u8 gGUA_Infrared_Sending;
extern uint16_t Detection_Delay_CNT;
extern struct ConfigData_Type ConfigData;

void IR_Pin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
	EXTI_ClearITPendingBit(EXTI_Line5);                             //PIR�ж�
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5); 
  EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  EXTI_ClearITPendingBit(EXTI_Line6);                            //IR�����ж�
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6); 
  EXTI_InitStructure.EXTI_Line=EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure); 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure);
}


/*TIM_Period--1000   TIM_Prescaler--71 -->�ж�����Ϊ50us*/
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    
	  TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period = 100;		 								/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72 ��������Ϊ1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    		/* �������жϱ�־ */
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);									/* ����ʱ�� */
    TIM_Cmd(TIM3, DISABLE);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
}

void TIM3_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  IrTimecnt++;
	
	
	if(IrTimecnt > 10000)  //�͵�ƽ����500ms�˳�
	{
		IrCode_idx = 0;
		IrTimecnt = 0;
		Ir_RecvComplete = false;
		Ir_RecvFlag = false;

		TIM_Cmd(TIM3, DISABLE);
		
		#ifdef  IR_RECV_DEBUG_INFO
      printf("������ճ�ʱ�˳�...\r\n");
    #endif
	}	
	
}

//******************************************************************************        
//name:             EXTI9_5_IRQHandler        
//introduce:        ������������ⲿ�ж�
//                  ���½��ش����� IO ���жϺ����У���Ҫʵ��ͳ�������½���֮���ʱ�䣬�������¼�������С�
//                  �½��ص�һ�δ���ʱ�������ǰ��ʱ���еļ���ֵ���Ա�ͳ��ʱ�䡣֮��ÿһ���½��ش����ͼ�¼�µ�ǰ����ֵ��
//                  Ȼ���ٶ������㡣�����ʱ����ͬ��ͷ��ʱ�������ڣ��������������㣬��ʾ���¿�ʼ�������ݡ�
//                  ��������ͬ��ͷ�� 32 ������֮�󣬱�ʾ������
//                  �����룺9ms�͵�ƽ+4.5ms�ߵ�ƽ              
//                  ����0 ��0.56ms�͵�ƽ+0.565ms�ߵ�ƽ
//                  ����1 ��0.56ms�͵�ƽ+1.685ms�ߵ�ƽ
//                  �����룺 0.263ms�͵�ƽ��Ϊ����λ
//                  ���ϵ�ƽ��׼�뷢����෴,��������Ϊ50us
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//****************************************************************************** 
void EXTI9_5_IRQHandler(void)
{
  uint16_t Ir_time;	
	
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		if(Ir_RecvFlag)
		{
				Ir_time = IrTimecnt; 
			
				if(Ir_time < 150 && Ir_time >= 120 ) // ���յ�������13.5ms,����12ms<13.5ms<15ms
				{
						IrCode_idx = 0;  // ��ʼ��������
				}			

				Ir_RecvBuff[IrCode_idx] = Ir_time;    // ��ȡ����ʱ��
				IrTimecnt = 0;                        // �������ʱ�䣬�Ա��´�ͳ��
				IrCode_idx++;                         // ���յ�һ�����ݣ�������1

				if(IrCode_idx == (GUA_INFRARED_FRAME_LEN * 8 + 1))       // ������յ�N������
				
				{
						TIM_Cmd(TIM3, DISABLE);					  
						Ir_RecvComplete = true;
						Ir_RecvFlag = false;
					  IrCode_idx = 0;
						IrTimecnt = 0;
					  
				}
		}
		else   // �½��ص�һ�δ���
		{
				IrCode_idx = 0;
				IrTimecnt = 0;
				Ir_RecvFlag = true;				
			  TIM_Cmd(TIM3, ENABLE);
		}

		EXTI_ClearITPendingBit(EXTI_Line6);  // ����жϱ�־
 }
	
 	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		PIR_DetectionFlag = true; 
		EXTI_ClearITPendingBit(EXTI_Line5);  // ����жϱ�־
	}
}

//******************************************************************************        
//name:             GUA_Infrared_Send_Init        
//introduce:        �����жϺ����н��ղ���¼�µ������������½���֮���ʱ�䣬�����Ǻ��������͵����ݡ�
//introduce:        �����Ҫ���ݺ���Э�飬�� 32 ��ʱ������жϣ��Ӷ���ú����������͵����ݡ�      
//                  ���������Ҫ�ں��������������ݺ�ִ�У���ͨ���жϽ�����ɱ�־λ receiveComplete ��ʵ�� 
//                  �����룺9ms�͵�ƽ+4.5ms�ߵ�ƽ
//                  ����0 ��0.56ms�͵�ƽ+0.565ms�ߵ�ƽ
//                  ����1 ��0.56ms�͵�ƽ+1.685ms�ߵ�ƽ
//                  �����룺0.263ms�͵�ƽ��Ϊ����λ
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************

uint8_t Infrared_DeCode(void)
{
  uint8_t i,j,idx=1; //idx ��1 ��ʼ��ʾ��ͬ��ͷ��ʱ�䲻����
  uint8_t data_temp,crc_temp;
	uint8_t InfraredRecv_code[GUA_INFRARED_FRAME_LEN];
	
	#ifdef  IR_RECV_DEBUG_INFO
     printf("\r\nremote_code:\r\n");
  #endif	
  
  Ir_RecvComplete = false;
	
	for(i=0; i<GUA_INFRARED_FRAME_LEN; i++)
  {
		for(j=0; j<8; j++)
		{				
			if(Ir_RecvBuff[idx] >=9 && Ir_RecvBuff[idx] < 13)   //��ʾ 0,(1.12ms=0.56ms+0.56ms)��,50us*19=0.95ms < ���յ��ĵ͵�ƽ���� < 50us*27=1.35ms
			{
					data_temp = 0;
			}
			else if(Ir_RecvBuff[idx] >=18 && Ir_RecvBuff[idx]<25) //��ʾ 1,(2.224ms=0.56ms+1.685ms),50us*36=1.8ms < ���յ��ĵ͵�ƽ���� < 50us*50=2.5ms
			{
					data_temp = 1;
			}
			
			InfraredRecv_code[i] <<= 1;
			InfraredRecv_code[i] |= data_temp;
			idx++;
		}
		
		if((i>1)&&(i<GUA_INFRARED_FRAME_LEN-1))
		{
		   crc_temp += InfraredRecv_code[i];
		}
		
		#ifdef  IR_RECV_DEBUG_INFO
			printf("0x%02X-",InfraredRecv_code[i]);
		#endif
			
  }
	
	#ifdef  IR_RECV_DEBUG_INFO
     printf("\r\n");
  #endif
	
	
	ConfigData.Data_CRC = ~crc_temp + 1;
	
	if(ConfigData.Data_CRC == InfraredRecv_code[GUA_INFRARED_DATA_LEN +2])  //CRCУ����ȷ
	{	
		if((InfraredRecv_code[0] == 0xAA)&(InfraredRecv_code[1] == 0x55))  //���²���
		{
			#ifdef  IR_RECV_DEBUG_INFO
				printf("ConfigData.Data_CRC: 0x%02X check correct!\r\n\r\n",ConfigData.Data_CRC);
			#endif
			
			ConfigData.Battery_Type          = InfraredRecv_code[2];
			ConfigData.Detect_Time1          = InfraredRecv_code[3];
			ConfigData.Power_SomeBody1       = InfraredRecv_code[4];
			ConfigData.Power_Nobody1         = InfraredRecv_code[5];
			ConfigData.Detect_Time2          = InfraredRecv_code[6];
			ConfigData.Power_SomeBody2       = InfraredRecv_code[7];
			ConfigData.Power_Nobody2         = InfraredRecv_code[8];
			ConfigData.Detect_Time3          = InfraredRecv_code[9];
			ConfigData.Power_SomeBody3       = InfraredRecv_code[10];
			ConfigData.Power_Nobody3         = InfraredRecv_code[11];
			ConfigData.Detection_Delay       = InfraredRecv_code[12];
			ConfigData.Optical_Ctl_Vol       = InfraredRecv_code[13];
			ConfigData.Optical_Ctl_Delay     = InfraredRecv_code[14];
			ConfigData.Load_RatedCurrent     = ((InfraredRecv_code[15]<<8)|InfraredRecv_code[16]) * 10;
			ConfigData.Auto_Power            = InfraredRecv_code[17];
			ConfigData.Charging_Freezing     = InfraredRecv_code[18];
			ConfigData.OverDischarge_Vol     = InfraredRecv_code[19];
			ConfigData.OverDischarge_BackVol = InfraredRecv_code[20];
			ConfigData.OverCharge_Vol        = InfraredRecv_code[21];
			ConfigData.OverCharge_BackVol    = InfraredRecv_code[22];
			ConfigData.Data_CRC              = InfraredRecv_code[23];
			
			SaveUser_ConfigData();
			
//			#ifdef  IR_RECV_DEBUG_INFO
//				printf("\r\nConfigData.Load_RatedCurrent: %d mA!\r\n",ConfigData.Load_RatedCurrent );
//			#endif
//			
  			
			Infrared_Send_NByte(0xAA, GUA_INFRARED_DATA_LEN, (u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;
		}
		else if((InfraredRecv_code[0] == 0xBB)&(InfraredRecv_code[1] == 0x44))  //���ز���
		{
		  GetDefault_ConfigData();
			Infrared_Send_NByte(0xBB, GUA_INFRARED_DATA_LEN,(u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;			
		}
		else if((InfraredRecv_code[0] == 0xDD)&(InfraredRecv_code[1] == 0x22))  //�ָ�����
		{
		  SetDefault_ConfigData();
		  GetDefault_ConfigData();
			Infrared_Send_NByte(0xDD, GUA_INFRARED_DATA_LEN,(u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;			
		}

	}
	else
	{
		#ifdef  IR_RECV_DEBUG_INFO
		 printf("ConfigData.Data_CRC: 0x%02X check incorrect!\r\n",ConfigData.Data_CRC);
		#endif
	}

	
	Ir_RecvComplete = false;
	
  return InfraredRecv_code[2]; 

}

//******************************************************************************        
//name:             Infrared_Recv_Init        
//introduce:        ������ճ�ʼ��     
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Recv_Init(void)
{  
  IR_Pin_init();
  TIM3_Configuration();
}



