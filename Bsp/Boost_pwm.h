#ifndef __BOOST_PWM_H
#define	__BOOST_PWM_H

#include "stm32f10x.h"


#define START_BOOST  TIM_Cmd(TIM1, ENABLE); TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);      
#define STOP_BOOST   TIM_Cmd(TIM1, DISABLE);TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);  //关闭后强制拉低

void Boost_PWM_Init(void);
void Start_Boot_PWM(void);
void Stop_Boot_PWM(void);

#endif /* __PWM_OUTPUT_H */

