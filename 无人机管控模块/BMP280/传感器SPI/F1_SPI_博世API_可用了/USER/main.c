/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��main.c
 * ����    ��NRF24L01����w�������ʵ��        
 * ʵ��ƽ̨��Ұ��STM32������	
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/
#include "stm32f10x.h"
#include "SPI_NRF.h"
#include "bmp280.h"
#include "usart1.h"
#include "bmp280_support.h"
#include "myspi.h"

u32 temp,press;
int main(void)
{
  u8 Bmp_ID=0; 
	uint8_t bmp280_id;  
  u32 maxtemp=0,mintemp=0,maxpress=0,minpress=0; 
  float hight;
  u64 t=0;
//  SPI_BMP_Init();
   
  /* ����1��ʼ�� */
	USART1_Config();
	/*��ȡ����ID��*/
//	Bmp_ID=SPI_BMP_ReadReg(0xd0);
//	while(Bmp_ID==0X58 )
//		printf("R/W is success,Bmp280_ID is 0x%x \r\n",Bmp_ID);
	Init_myspi();
	for(t=10;t>0;t--)
        bmp280_data_readout_template(&temp,&press);
	maxtemp=mintemp=temp;
    maxpress=minpress=press;

    while(1)
    {
			
				for(t=10;t>0;t--)
        bmp280_data_readout_template(&temp,&press);
	maxtemp=mintemp=temp;
    maxpress=minpress=press;
			
       if(temp>maxtemp)
            maxtemp=temp;
        if(temp<mintemp)
            mintemp=temp;
        if(press>maxpress)
            maxpress=press;
        if(press<minpress)
            minpress=press;
        hight = 0.01*press;
        printf("\r\n\r\n");
        printf("��%lld��������������\r\n",++t);
        printf("��ǰ�¶ȣ�%.2f DegC   ��ǰѹǿ=%.2f hPa  ���Σ�%.2f m\r\n",\
        0.01*temp,0.01*press,bmp280PressureToAltitude(&hight));
        printf("����¶ȣ�%.2f DegC   ����¶ȣ�%.2f DegC       ���ѹǿ��%.2f hPa   ��Сѹǿ��%.2f hPa\r\n",\
        0.01*maxtemp,0.01*mintemp,0.01*maxpress,0.01*minpress);
        printf(" ����¶� - ��С�¶ȣ�%.2f DegC   ���ѹǿ - ��Сѹǿ��%.2f hPa\r\n",\
        0.01*(maxtemp-mintemp),0.01*(maxpress-minpress));
        us_Dealy(500);us_Dealy(500);
    }
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

