/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  ：main.c
 * 描述    ：NRF24L01无线w传输测试实验        
 * 实验平台：野火STM32开发板	
 * 库版本  ：ST3.5.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：http://www.amobbs.com/forum-1008-1.html
 * 淘宝    ：http://firestm32.taobao.com
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
   
  /* 串口1初始化 */
	USART1_Config();
	/*读取器件ID号*/
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
        printf("第%lld次运行数据如下\r\n",++t);
        printf("当前温度：%.2f DegC   当前压强=%.2f hPa  海拔：%.2f m\r\n",\
        0.01*temp,0.01*press,bmp280PressureToAltitude(&hight));
        printf("最高温度：%.2f DegC   最低温度：%.2f DegC       最大压强：%.2f hPa   最小压强：%.2f hPa\r\n",\
        0.01*maxtemp,0.01*mintemp,0.01*maxpress,0.01*minpress);
        printf(" 最大温度 - 最小温度：%.2f DegC   最大压强 - 最小压强：%.2f hPa\r\n",\
        0.01*(maxtemp-mintemp),0.01*(maxpress-minpress));
        us_Dealy(500);us_Dealy(500);
    }
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

