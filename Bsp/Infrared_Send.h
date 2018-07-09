
//******************************************************************************              
//name:             Infrared_Send.h             
//introduce:        ���⴫��������������ͷ�ļ�      
//author:                             
//email:                                      
//changetime:       2016.11.16      
//******************************************************************************    
#include "stm32f10x.h"

#ifndef _INFRARED_SEND_H_
#define _INFRARED_SEND_H_

/*********************�궨��************************/  
//���ⷢ������
#define GUA_INFRARED_SEND_RCC          RCC_APB2Periph_GPIOB
#define GUA_INFRARED_SEND_PORT         GPIOB
#define GUA_INFRARED_SEND_PIN          GPIO_Pin_7
#define GUA_INFRARED_SEND_MODE         GPIO_Mode_Out_PP_High_Fast

//�����IO��ƽ��
#define GUA_INFRARED_ON                Bit_SET             //�ߵ�ƽ������
#define GUA_INFRARED_OFF               Bit_RESET           //�͵�ƽ���ر�

//�����־��
#define GUA_INFRARED_FLAG_OFF          0               //���Ͷ�Ϊ�ߵ�ƽ�����ն�Ϊ�͵�ƽ
#define GUA_INFRARED_FLAG_ON           1               //���Ͷ�Ϊ�͵�ƽ�����ն�Ϊ�ߵ�ƽ

#define GUA_INFRARED_DATA_LEN          22               //Ҫ���͵ĺ������ݳ��ȣ�������1�ֽ��û��롢1�ֽ��û����롢��1�ֽ�CRCУ��

#define GUA_INFRARED_FRAME_LEN         (GUA_INFRARED_DATA_LEN + 3)   //Ҫ���͵ĺ���֡���ȣ�����1Byte�û��롢1Byte���û����롢��1ByteCRCУ�顢1�ֽڵĶ�������

//#define GUA_USER_CODE   0xAA  

#define INFRARED_SEND_DEBUG_INFO 0x01    //�������ڴ�ӡ������Ϣ���������Ҫ�����θ���

/*********************�ⲿ����************************/  
extern unsigned char gGUA_Infrared_Flag;
extern volatile unsigned short gGUA_Infrared_Count;

/*********************�ⲿ��������************************/  
extern void Infrared_Send(u32 nGUA_Data);
extern void Infrared_Send_Init(void);

void Infrared_Send_NByte(u8 GUA_USER_CODE,u8 DataLengh, u8 *nGUA_Data);

#endif

