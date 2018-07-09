/******************** (C) COPYRIGHT 2012  Team **************************
 * �ļ���  ��flash.c
 * ����    ��         
 * ʵ��ƽ̨��STM32F103C8T6
   			
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "flash.h"


//��ȡָ����ַ�İ���(16λ����)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*)address; 
}

//��ȡָ����ַ��ȫ��(32λ����)
uint32_t FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
	
  temp1 =*(__IO uint16_t*)address; 
  temp2 =*(__IO uint16_t*)(address+2);
	
  return (temp2<<16)+temp1;
}

//��ָ����ַ��ʼ��ȡ�������
void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead)
{
  uint16_t dataIndex;
	
  for(dataIndex = 0; dataIndex < countToRead; dataIndex++)
  {
    readData[dataIndex] = FLASH_ReadHalfWord(startAddress + dataIndex * 2);
  }
}


//��ָ����ַ��ʼд��������
void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
{
  uint16_t dataIndex;
  uint32_t offsetAddress;
  uint32_t sectorPosition;
  uint32_t sectorStartAddress;
	
  if(startAddress<FLASH_BASE||((startAddress+countToWrite*2)>=(FLASH_BASE+1024*FLASH_SIZE)))
  {
    return;//�Ƿ���ַ
  }
  
  FLASH_Unlock();         //����д����
  
  offsetAddress = startAddress-FLASH_BASE;               //����ȥ��0x08000000���ʵ��ƫ�Ƶ�ַ
  sectorPosition = offsetAddress/SECTOR_SIZE;            //����������ַ������STM32F103VET6Ϊ0~255
  
  sectorStartAddress = sectorPosition*SECTOR_SIZE+FLASH_BASE;    //��Ӧ�������׵�ַ

  FLASH_ErasePage(sectorStartAddress);//�����������
   
  
  for(dataIndex=0; dataIndex<countToWrite; dataIndex++)
  {
    FLASH_ProgramHalfWord(startAddress+dataIndex*2,writeData[dataIndex]);
  }
  
  FLASH_Lock();//����д����
}

/******************* (C) COPYRIGHT 2012  Team *****END OF FILE************/
