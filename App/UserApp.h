#ifndef __USERAPP_H
#define	__USERAPP_H

#include "stdbool.h"
#include "Pvd.h"
#include "Adc.h"
#include "Usart1.h"
#include "SysTick.h"
#include "Boost_pwm.h"
#include "Charge_pwm.h"
#include "Control_GPIO.h"
#include "Infrared_Send.h"
#include "Infrared_Recv.h"
#include "EEpromInFlash.h"


#define USERAPP_DEBUG_INFO 0x01  //�������ڴ�ӡ������Ϣ���������Ҫ�����θ���

#define SOLAR_BELOW_OPTICAL_VOL    0x01    //̫���������ѹ���ڹ�ص㣬����ҹ��ģʽ
#define SOLAR_ABOVE_BATTERY_VOL    0x02    //̫���������ѹ���ڵ�ص�ѹ�����Գ��

#define BATTERY_OVER_DISCHARGE     0x01    //��ع���
#define BATTERY_DISCHARGE_BACK     0x02    //���ŷ���
#define BATTERY_CHARGE_BACK        0x03    //��ع���
#define BATTERY_OVER_CHARGE        0x04    //���巵��

#define LOAD_OVERRATED     0x01    //���ع���
#define LOAD_OPEN          0x02    //���ؿ�·
#define LOAD_RATED         0x03    //���ض
#define LOAD_ADJ           0x04    //���ص���

#define STATE_OK           0x0F    //״̬����
#define STATE_ERR          0x00    //״̬�쳣

#define MODE_STANDBY       0x01    //����ģʽ
#define MODE_POWERONTEST   0x02    //��������ģʽ
#define MODE_IDLE          0x03    //�������ģʽ
#define MODE_LIGHTCTL      0x04    //����ӳ�ģʽ
#define MODE_TIMESLOT1     0x05    //��Ӧʱ���1ģʽ
#define MODE_TIMESLOT2     0x06    //��Ӧʱ���2ģʽ
#define MODE_TIMESLOT3     0x07    //��Ӧʱ���3ģʽ


#define Addr_Config_Tag            0x00       //���ñ��
#define Addr_Battery_Type          0x01       //�������:0x01:�12,0x02:�24,  ��λ:V,  Ĭ��0x01
#define Addr_Detect_Time1          0x02       //��Ӧʱ��1:0~15H,����1H,        ��λ:H,  Ĭ��0x02
#define Addr_Power_SomeBody1       0x03       //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%
#define Addr_Power_Nobody1         0x04       //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%
#define Addr_Detect_Time2          0x05       //��Ӧʱ��2:0~15H,����1H,        ��λ:H,  Ĭ��0x03   3H
#define Addr_Power_SomeBody2       0x06       //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%
#define Addr_Power_Nobody2         0x07       //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%
#define Addr_Detect_Time3          0x08       //��Ӧʱ��3:0~15H,����1H,        ��λ:H,  Ĭ��0x08   8H
#define Addr_Power_SomeBody3       0x09       //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x46   70%
#define Addr_Power_Nobody3         0x0A       //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x32   50%
#define Addr_Detection_Delay       0x0B       //���˸�Ӧ�ӳ�ʱ��:0~255s,����1s, ��λ:s, Ĭ��0x1E   30S
#define Addr_Optical_Ctl_Vol       0x0C       //��ص��ѹ:5~11V,����1V,       ��λ:V,  Ĭ��0x05
#define Addr_Optical_Ctl_Delay     0x0D       //����ӳ�:0~50min,����1min,     ��λ:min,Ĭ��0x05
#define Addr_Load_Current          0x0E       //���ص���:0~6000mA,����10mA,    ��λ:mA, Ĭ��0x012C ,300mA
#define Addr_Auto_Power            0x0F       //���ܹ���:0x01:����,0x02�ر�             Ĭ��0x01
#define Addr_Charging_Freezing     0x10       //���³��:0x01:����,0x02��ֹ             Ĭ��0x02
#define Addr_OverDischarge_Vol     0x11       //���ű�����ѹ:7500~17000,����100mV,      ��λ:mV, Ĭ��0x2710 ,10V
#define Addr_OverDischarge_BackVol 0x12       //���ŷ��ص�ѹ:7500~17000,����100mV,      ��λ:mV, Ĭ��0x2EE0 ,12V
#define Addr_OverCharge_Vol        0x13       //���䱣����ѹ:7500~17000,����100mV,      ��λ:mV, Ĭ��0x3908 ,14.6V
#define Addr_OverCharge_BackVol    0x14       //���䷵�ص�ѹ:7500~17000,����100mV,      ��λ:mV, Ĭ��0x3520 ,13.6V
#define Addr_Data_CRC              0x20       //����CRC

//struct ConfigData_Type
//{
//	uint8_t Config_Tag;             //���ñ��
//	uint8_t Battery_Type;
//	uint8_t Detect_Time1;
//	uint8_t Power_SomeBody1;
//	uint8_t Power_Nobody1;
//	uint8_t Detect_Time2;
//	uint8_t Power_SomeBody2;
//	uint8_t Power_Nobody2;
//	uint8_t Detect_Time3;
//	uint8_t Power_SomeBody3;
//	uint8_t Power_Nobody3;
//	uint8_t Detection_Delay;
//	uint8_t Optical_Ctl_Vol;
//	uint8_t Optical_Ctl_Delay;
//	uint16_t Load_RatedCurrent;
//	uint8_t Auto_Power;
//	uint8_t Charging_Freezing;
//	uint16_t OverDischarge_Vol;
//	uint16_t OverDischarge_BackVol;
//	uint16_t OverCharge_Vol;
//	uint16_t OverCharge_BackVol;
//	uint8_t Data_CRC;
//	uint8_t Contorller_Mode;
//};
struct ConfigData_Type
{
	uint8_t Config_Tag;             //���ñ��
	uint8_t Battery_Type;
	uint8_t Detect_Time1;
	uint8_t Power_SomeBody1;
	uint8_t Power_Nobody1;
	uint8_t Detect_Time2;
	uint8_t Power_SomeBody2;
	uint8_t Power_Nobody2;
	uint8_t Detect_Time3;
	uint8_t Power_SomeBody3;
	uint8_t Power_Nobody3;
	uint8_t Detection_Delay;
	uint8_t Optical_Ctl_Vol;
	uint8_t Optical_Ctl_Delay;
	uint16_t Load_RatedCurrent;
	uint8_t Auto_Power;
	uint8_t Charging_Freezing;
	uint8_t OverDischarge_Vol;
	uint8_t OverDischarge_BackVol;
	uint8_t OverCharge_Vol;
	uint8_t OverCharge_BackVol;
	uint8_t Data_CRC;
	uint8_t Contorller_Mode;
};

extern __IO uint16_t ADC_FilterValue[8];


void Hardware_Init(void);
void System_Init(void);

u16 Check_Solar_InputStatus(void);
u16 Check_Battery_Status(void);

u16 Check_LoadCurrent_Status(void);
u16 Check_BoostCurrent_Status(void);
u16 Check_SolarCurrent_Status(void);
u16 Is_ExternTemp_Frezing(void);

void ConfigData_Init(void);
void SaveUser_ConfigData(void);
void GetDefault_ConfigData(void);
void SetDefault_ConfigData(void);

void Check_PIR_Status(void);
void Check_Infrared_Recv_Status(void);

void Is_SomeBody_Detection(void);
void Is_Infrared_Recv(void);

u16 Is_DetectTimeSlot(void);
u16 Is_LightControlDelayEnd(void);
u16 Is_LoadCurrent_Rated(void);

void WorkInPowerOnTest(void);
void WorkInStandby(void);
void WorkInIdleMode(void);
void WorkInLightControlDelay(void);
void WorkInTimeSlot1(void);
void WorkInTimeSlot2(void);
void WorkInTimeSlot3(void);

#endif /* __USERAPP_H */
