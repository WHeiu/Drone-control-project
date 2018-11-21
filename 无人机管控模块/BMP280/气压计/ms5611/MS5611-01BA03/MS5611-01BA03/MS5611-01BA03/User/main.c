#include "allheader.h"
extern defMS5611data ms5611data;
int main(void)
{	
	SystemInit();			//系统初始化
	delay_init(72);	     	//延时初始化
	NVIC_Configuration(); 	//中断初始化
 	uart_init(9600);		//串口初始化
	
	printf("System Init OK\n");
	
	IIC_Init();		//MS561101BA初始化
	
	MS561101BA_Init(); //MS561101BA初始化
	
	printf("Hardware Init OK\n");
	
	while(1)
	{	
/*********************************大气压测量**********************************/
		MS561101BA_readdata(&ms5611data);
		printf("%d\n",ms5611data.temperature);
/***************************************************************************/
	}
}
