#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"

#define ADC_CNV_NUM 20

//#define ADC_DEBUG_INFO 0x01  //�������ڴ�ӡ������Ϣ���������Ҫ�����θ���

void ADC_ValueAVGFilter(void);
void ADC1_Init(void);

u16 GetSTM32Temp(void);
u16 GetSTM32Vref(void);
float GetBaterryVolt(void);
float GetSolarVolt(void);
u16 GetSolarCurrent(void);
u16 GetBoostCurrent(void);
u16 GetLoadCurrent(void);


#endif /* __ADC_H */

