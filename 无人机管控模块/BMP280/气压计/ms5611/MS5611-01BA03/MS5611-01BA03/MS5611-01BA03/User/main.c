#include "allheader.h"
extern defMS5611data ms5611data;
int main(void)
{	
	SystemInit();			//ϵͳ��ʼ��
	delay_init(72);	     	//��ʱ��ʼ��
	NVIC_Configuration(); 	//�жϳ�ʼ��
 	uart_init(9600);		//���ڳ�ʼ��
	
	printf("System Init OK\n");
	
	IIC_Init();		//MS561101BA��ʼ��
	
	MS561101BA_Init(); //MS561101BA��ʼ��
	
	printf("Hardware Init OK\n");
	
	while(1)
	{	
/*********************************����ѹ����**********************************/
		MS561101BA_readdata(&ms5611data);
		printf("%d\n",ms5611data.temperature);
/***************************************************************************/
	}
}
