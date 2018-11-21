/*******************************************************************************
  * �ļ�����: STM32F103 BMP280����
	* ������: ����ѹ���¶ȡ��߶�
  * �������ߣ��Թ���
  * �汾��ţ�V1.0
  * ��дʱ��: 2018.05.13
  * �޸�ʱ��:
  ******************************************************************************
  * �޸ļ�¼:
  * 1�������ѹ�¶ȱ仯������IIR�˲��������ѹ�仯���ң�ȥ���˲��� 2018.05.13
	*
  ******************************************************************************
  */ 
	
/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "rs485.h"
#include "tftlcd.h"
#include "key.h"
#include "stm32_flash.h"
#include "24cxx.h" 
#include "myiic.h"
#include "bmp280.h"

/* main ----------------------------------------------------------------------*/
 int main(void)
 {	 
	float bmp280_temp;
	float bmp280_press;
  float high;
	u8 i=0;
//	u16 p,t,h;
	u16 t;
	u32 press,press1,press2;
	
	SysTick_Init(72);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�ж����ȼ����� ��2��
	LED_Init();
	USART1_Init(9600);
	RS485_Init(9600);
	TFTLCD_Init();			//LCD��ʼ��
	KEY_Init();
	bmp280Init();       //BMP280��ʼ��
	
	FRONT_COLOR=BLACK;
	LCD_ShowString(20,50,200,16,16,"Puzhong STM32");
	LCD_ShowString(20,70,200,16,16,"PengZhuang IIC TEST");	
	LCD_ShowString(20,90,200,16,16,"BY ZhaoGuiSheng");
	LCD_ShowString(20,110,200,16,16,"2018/5/12");		
	 
 	FRONT_COLOR=BLUE;//��������Ϊ��ɫ	  
	LCD_ShowString(20,150,200,16,16,"The temperature is    deg");	
	LCD_ShowString(20,170,200,16,16,"The airpress is");	
	LCD_ShowString(160,205,200,16,16,"pa");	  
	FRONT_COLOR=RED;  //��������Ϊ��ɫ		
	
	LCD_ShowChar(20+9*12,200,'.',24,0);
	
	while(1)
	{
			bmp280GetData(&bmp280_press,&bmp280_temp,&high);
			delay_ms(200);

//		p=bmp280_press;
		  press=bmp280_press*1000;
		  press1=bmp280_press*100;
		  press2=press%10;
			t=bmp280_temp;
 //		h=high;                   //��Ŀ����Ҫʹ�ø߶�ֵ		
		   
		  LCD_ShowxNum(20+19*8,150,t,2,16,0);
		  LCD_ShowxNum(20+3*12,200,press1,6,24,0);
		  LCD_ShowxNum(20+10*12,200,press2,1,24,0);
		
			i++;
			if(i==20)
			{
				led1=!led1;//��ʾϵͳ��������	
				i=0;
			}		   
	}
}
