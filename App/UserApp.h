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


#define USERAPP_DEBUG_INFO 0x01  //开启串口打印调试信息，如果不需要，屏蔽该行

#define SOLAR_BELOW_OPTICAL_VOL    0x01    //太阳板输入电压低于光控点，进入夜晚模式
#define SOLAR_ABOVE_BATTERY_VOL    0x02    //太阳板输入电压高于电池电压，可以充电

#define BATTERY_OVER_DISCHARGE     0x01    //电池过放
#define BATTERY_DISCHARGE_BACK     0x02    //过放返回
#define BATTERY_CHARGE_BACK        0x03    //电池过充
#define BATTERY_OVER_CHARGE        0x04    //过冲返回

#define LOAD_OVERRATED     0x01    //负载过流
#define LOAD_OPEN          0x02    //负载开路
#define LOAD_RATED         0x03    //负载额定
#define LOAD_ADJ           0x04    //负载调整

#define STATE_OK           0x0F    //状态正常
#define STATE_ERR          0x00    //状态异常

#define MODE_STANDBY       0x01    //待机模式
#define MODE_POWERONTEST   0x02    //开机测试模式
#define MODE_IDLE          0x03    //白天空闲模式
#define MODE_LIGHTCTL      0x04    //光控延迟模式
#define MODE_TIMESLOT1     0x05    //感应时间段1模式
#define MODE_TIMESLOT2     0x06    //感应时间段2模式
#define MODE_TIMESLOT3     0x07    //感应时间段3模式


#define Addr_Config_Tag            0x00       //配置标记
#define Addr_Battery_Type          0x01       //电池类型:0x01:锂12,0x02:锂24,  单位:V,  默认0x01
#define Addr_Detect_Time1          0x02       //感应时间1:0~15H,步长1H,        单位:H,  默认0x02
#define Addr_Power_SomeBody1       0x03       //有人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%
#define Addr_Power_Nobody1         0x04       //无人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%
#define Addr_Detect_Time2          0x05       //感应时间2:0~15H,步长1H,        单位:H,  默认0x03   3H
#define Addr_Power_SomeBody2       0x06       //有人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%
#define Addr_Power_Nobody2         0x07       //无人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%
#define Addr_Detect_Time3          0x08       //感应时间3:0~15H,步长1H,        单位:H,  默认0x08   8H
#define Addr_Power_SomeBody3       0x09       //有人感应功率3:0~100%,步长10%,  单位:%,  默认0x46   70%
#define Addr_Power_Nobody3         0x0A       //无人感应功率3:0~100%,步长10%,  单位:%,  默认0x32   50%
#define Addr_Detection_Delay       0x0B       //有人感应延迟时间:0~255s,步长1s, 单位:s, 默认0x1E   30S
#define Addr_Optical_Ctl_Vol       0x0C       //光控点电压:5~11V,步长1V,       单位:V,  默认0x05
#define Addr_Optical_Ctl_Delay     0x0D       //光控延迟:0~50min,步长1min,     单位:min,默认0x05
#define Addr_Load_Current          0x0E       //负载电流:0~6000mA,步长10mA,    单位:mA, 默认0x012C ,300mA
#define Addr_Auto_Power            0x0F       //智能功率:0x01:开启,0x02关闭             默认0x01
#define Addr_Charging_Freezing     0x10       //零下充电:0x01:允许,0x02禁止             默认0x02
#define Addr_OverDischarge_Vol     0x11       //过放保护电压:7500~17000,步长100mV,      单位:mV, 默认0x2710 ,10V
#define Addr_OverDischarge_BackVol 0x12       //过放返回电压:7500~17000,步长100mV,      单位:mV, 默认0x2EE0 ,12V
#define Addr_OverCharge_Vol        0x13       //过充保护电压:7500~17000,步长100mV,      单位:mV, 默认0x3908 ,14.6V
#define Addr_OverCharge_BackVol    0x14       //过充返回电压:7500~17000,步长100mV,      单位:mV, 默认0x3520 ,13.6V
#define Addr_Data_CRC              0x20       //数据CRC

//struct ConfigData_Type
//{
//	uint8_t Config_Tag;             //配置标记
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
	uint8_t Config_Tag;             //配置标记
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
