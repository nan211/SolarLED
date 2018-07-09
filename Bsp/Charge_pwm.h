#ifndef __CHARGE_PWM_H
#define	__CHARGE_PWM_H

#include "stm32f10x.h"

#define DISABLE_DIRECT_CHARGE TIM_Cmd(TIM4, DISABLE);TIM_ForcedOC3Config(TIM4, TIM_ForcedAction_InActive);    //��ʹ��PWM���ģʽ��ǿ������,�ضϳ������
#define ENABLE_DIRECT_CHARGE  TIM_Cmd(TIM4, DISABLE);TIM_ForcedOC3Config(TIM4, TIM_ForcedAction_Active);  //��ʹ��PWM���ģʽ��ǿ������,�򿪳������

#define DISABLE_PWM_CHARGE TIM_Cmd(TIM4, DISABLE);                                                          //��ʹ��PWM���ģʽ��ǿ������,�ضϳ������
#define ENABLE_PWM_CHARGE  TIM_Cmd(TIM4, ENABLE);                                                          //��ʹ��PWM���ģʽ��ǿ������,�򿪳������



void Charge_PWM_Init(void);

#endif /* __PWM_OUTPUT_H */

