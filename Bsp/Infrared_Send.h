
//******************************************************************************              
//name:             Infrared_Send.h             
//introduce:        红外传感器接收驱动的头文件      
//author:                             
//email:                                      
//changetime:       2016.11.16      
//******************************************************************************    
#include "stm32f10x.h"

#ifndef _INFRARED_SEND_H_
#define _INFRARED_SEND_H_

/*********************宏定义************************/  
//红外发送引脚
#define GUA_INFRARED_SEND_RCC          RCC_APB2Periph_GPIOB
#define GUA_INFRARED_SEND_PORT         GPIOB
#define GUA_INFRARED_SEND_PIN          GPIO_Pin_7
#define GUA_INFRARED_SEND_MODE         GPIO_Mode_Out_PP_High_Fast

//红外的IO电平宏
#define GUA_INFRARED_ON                Bit_SET             //高电平触开启
#define GUA_INFRARED_OFF               Bit_RESET           //低电平触关闭

//红外标志宏
#define GUA_INFRARED_FLAG_OFF          0               //发送端为高电平，接收端为低电平
#define GUA_INFRARED_FLAG_ON           1               //发送端为低电平，接收端为高电平

#define GUA_INFRARED_DATA_LEN          22               //要发送的红外数据长度，不包括1字节用户码、1字节用户反码、和1字节CRC校验

#define GUA_INFRARED_FRAME_LEN         (GUA_INFRARED_DATA_LEN + 3)   //要发送的红外帧长度，包括1Byte用户码、1Byte用用户反码、和1ByteCRC校验、1字节的多余数据

//#define GUA_USER_CODE   0xAA  

#define INFRARED_SEND_DEBUG_INFO 0x01    //开启串口打印调试信息，如果不需要，屏蔽该行

/*********************外部变量************************/  
extern unsigned char gGUA_Infrared_Flag;
extern volatile unsigned short gGUA_Infrared_Count;

/*********************外部函数声明************************/  
extern void Infrared_Send(u32 nGUA_Data);
extern void Infrared_Send_Init(void);

void Infrared_Send_NByte(u8 GUA_USER_CODE,u8 DataLengh, u8 *nGUA_Data);

#endif

