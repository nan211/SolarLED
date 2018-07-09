#ifndef __CHARGE_PWM_H
#define	__CHARGE_PWM_H

#include "stm32f10x.h"

#define DISABLE_DIRECT_CHARGE TIM_Cmd(TIM4, DISABLE);TIM_ForcedOC3Config(TIM4, TIM_ForcedAction_InActive);    //不使用PWM充电模式，强行拉低,关断充电输入
#define ENABLE_DIRECT_CHARGE  TIM_Cmd(TIM4, DISABLE);TIM_ForcedOC3Config(TIM4, TIM_ForcedAction_Active);  //不使用PWM充电模式，强行拉高,打开充电输入

#define DISABLE_PWM_CHARGE TIM_Cmd(TIM4, DISABLE);                                                          //不使用PWM充电模式，强行拉低,关断充电输入
#define ENABLE_PWM_CHARGE  TIM_Cmd(TIM4, ENABLE);                                                          //不使用PWM充电模式，强行拉高,打开充电输入



void Charge_PWM_Init(void);

#endif /* __PWM_OUTPUT_H */

