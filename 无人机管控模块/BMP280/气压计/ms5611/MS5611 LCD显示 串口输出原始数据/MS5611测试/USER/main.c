#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "key.h"  
#include "myiic.h"
#include "MS5611.h"

 int main(void)
 { 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 //���ڳ�ʼ��Ϊ9600
	LED_Init();		  		 //��ʼ����LED���ӵ�Ӳ���ӿ�
		 	
//  LCD_Show();          //��ʼ��LCD������������ɫ�� ��ʾ��Ҫ��������
	
	delay_ms(1000);
	IIC_Init();	         //��ʼ��IIC PC11 PC12����
	delay_ms(100);
	MS561101BA_RESET();	 // Reset Device  ��λMS5611
	delay_ms(100);       //��λ����ʱ��ע�������ʱ��һ����Ҫ�ģ��������̵��ƺ���������20ms��
	MS5611_init();	     //��ʼ��MS5611
	 
	while(1)
	{
		delay_ms(200);
		
   	MS561101BA_GetTemperature();//��ȡ�¶�
		MS561101BA_getPressure();   //��ȡ����ѹ
		
		LCD_MS5611_DataShow();      //LCD��ʾ�ִ�������ݣ���ʾ
	}
}
