#ifndef __FLASH_H
#define	__FLASH_H

#include "stm32f10x.h"

#define FLASH_SIZE 512          //��ѡMCU��FLASH������С(��λΪK)

#if FLASH_SIZE<256
  #define SECTOR_SIZE           1024    //�ֽ�
#else 
  #define SECTOR_SIZE           2048    //�ֽ�
#endif


#endif /* __FLASH_H */
