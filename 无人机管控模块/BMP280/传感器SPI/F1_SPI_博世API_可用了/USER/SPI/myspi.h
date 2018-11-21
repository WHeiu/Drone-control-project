#ifndef __MYSPI_H__
#define __MYSPI_H__
#include "stm32f10x.h"
//SPI输入  
#define         READ_MISO   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) //SDO 
// //
#define         CE_L      GPIO_ResetBits(GPIOA, GPIO_Pin_1) //SCB
//
#define         CE_H      GPIO_SetBits(GPIOA, GPIO_Pin_1)   
//SPI输出  
#define         MOSI_L  GPIO_ResetBits(GPIOA , GPIO_Pin_7)  //SDI  SDA
#define         MOSI_H  GPIO_SetBits(GPIOA , GPIO_Pin_7)     
//SPI时钟  
#define         SCK_L   GPIO_ResetBits(GPIOA , GPIO_Pin_5)  //SCK SCL
#define         SCK_H   GPIO_SetBits(GPIOA , GPIO_Pin_5)  


/*
功能：初始化spi接口
*/
void Init_myspi(void)  ;
/*
功能：主、从机交换一个字节
*/
unsigned char spixfer(unsigned char x) ;

#endif
