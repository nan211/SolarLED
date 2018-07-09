#ifndef __FLASH_H
#define	__FLASH_H

#include "stm32f10x.h"

#define FLASH_SIZE 512          //所选MCU的FLASH容量大小(单位为K)

#if FLASH_SIZE<256
  #define SECTOR_SIZE           1024    //字节
#else 
  #define SECTOR_SIZE           2048    //字节
#endif


#endif /* __FLASH_H */
