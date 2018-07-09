
//******************************************************************************              
//name:             Infrared_Recv.h             
//introduce:        红外传感器接收驱动的头文件      
//author:                             
//email:                                      
//changetime:       2016.11.16      
//******************************************************************************    
#include "stm32f10x.h"
#include "stdbool.h"

#ifndef _INFRARED_RECV_H_
#define _INFRARED_RECV_H_

#define IR_RECV_DEBUG_INFO 0x01  //开启串口打印调试信息，如果不需要，屏蔽该行

void Infrared_Recv_Init(void);
uint8_t Infrared_DeCode(void);



#endif
