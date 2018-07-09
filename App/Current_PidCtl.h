#ifndef __CURRENT_PIDCTL_H
#define	__CURRENT_PIDCTL_H

#include "stm32f10x.h"

void CurrentPID_init(void);
float CurrentPID_Realize(float SetCurrent);

#endif

