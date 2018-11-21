
#include "myspi.h"
/*
���ܣ���ʼ��spi�ӿ�
*/
void Init_myspi(void)  
{   
    GPIO_InitTypeDef GPIO_InitStructure;              
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //ʹ��GPIO ��ʱ��  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //�������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
    CE_H;           //��ʼ��ʱ������  
      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
}
/*
���ܣ������ӻ�����һ���ֽ�
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

