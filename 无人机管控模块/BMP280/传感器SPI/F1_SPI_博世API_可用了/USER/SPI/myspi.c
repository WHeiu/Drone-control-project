
#include "myspi.h"
/*
功能：初始化spi接口
*/
void Init_myspi(void)  
{   
    GPIO_InitTypeDef GPIO_InitStructure;              
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //使能GPIO 的时钟  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
    CE_H;           //初始化时先拉高  
      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
}
/*
功能：主、从机交换一个字节
*/
uint8_t spixfer(uint8_t x) 
{ 
  int i;
  uint8_t reply = 0;
  for (i=7; i>=0; i--) {
    reply <<= 1;
    SCK_L;
    if(x & (1<<i))
    {
        MOSI_H;
    }
    else 
    {
        MOSI_L;
    }
    SCK_H;//
    if (READ_MISO) 
      reply |= 1;
  }
  return reply;
}

