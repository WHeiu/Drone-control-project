#ifndef __MYSPI_H__
#define __MYSPI_H__
#include "stm32f10x.h"
//SPI����  
#define         READ_MISO   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) //SDO 
// //
#define         CE_L      GPIO_ResetBits(GPIOA, GPIO_Pin_1) //SCB
//
#define         CE_H      GPIO_SetBits(GPIOA, GPIO_Pin_1)   
//SPI���  
#define         MOSI_L  GPIO_ResetBits(GPIOA , GPIO_Pin_7)  //SDI  SDA
#define         MOSI_H  GPIO_SetBits(GPIOA , GPIO_Pin_7)     
//SPIʱ��  
#define         SCK_L   GPIO_ResetBits(GPIOA , GPIO_Pin_5)  //SCK SCL
#define         SCK_H   GPIO_SetBits(GPIOA , GPIO_Pin_5)  


/*
���ܣ���ʼ��spi�ӿ�
*/
void Init_myspi(void)  ;
/*
���ܣ������ӻ�����һ���ֽ�
*/
unsigned char spixfer(unsigned char x) ;

#endif
