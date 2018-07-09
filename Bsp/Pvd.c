#include "Pvd.h"
#include "Control_Gpio.h"
extern void Stop_Boot_PWM(void);

//extern bool Boost_Working = false;
void PWR_PVD_Init(void) 
{   
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
     
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
 
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;           //使能PVD所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);
     
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;             //PVD连接到中断线16上
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     //使用中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //电压低于阀值时产生中断
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               //使能中断线
    EXTI_Init(&EXTI_InitStructure);                         //初始
     
    PWR_PVDLevelConfig(PWR_PVDLevel_2V8);//设定监控阀值
    PWR_PVDCmd(ENABLE);//使能PVD     
}
void PVD_IRQHandler(void)
{ 
    EXTI_ClearITPendingBit(EXTI_Line16);//清中断
     
//    num = BKP_ReadBackupRegister(BKP_DR10);
//    num++;
	
     ENABLE_OVERLOAD;
	   Stop_Boot_PWM();
	
//	   Boost_Working = false;
    //用户添加紧急处理代码处
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);//使能PWR和BKP外设时钟
//    PWR_BackupAccessCmd(ENABLE);//使能后备寄存器访问
// 
//    BKP_WriteBackupRegister(BKP_DR10, (u8)num);//启动界面 
	
}
