
//******************************************************************************              
//name:             Infrared_Recv.h             
//introduce:        ���⴫��������������ͷ�ļ�      
//author:                             
//email:                                      
//changetime:       2016.11.16      
//******************************************************************************    
#include "stm32f10x.h"
#include "stdbool.h"

#ifndef _INFRARED_RECV_H_
#define _INFRARED_RECV_H_

#define IR_RECV_DEBUG_INFO 0x01  //�������ڴ�ӡ������Ϣ���������Ҫ�����θ���

void Infrared_Recv_Init(void);
uint8_t Infrared_DeCode(void);



#endif
