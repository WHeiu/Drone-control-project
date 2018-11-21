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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 //串口初始化为9600
	LED_Init();		  		 //初始化与LED连接的硬件接口
		 	
//  LCD_Show();          //初始化LCD，设置字体颜色， 显示必要参数符号
	
	delay_ms(1000);
	IIC_Init();	         //初始化IIC PC11 PC12口子
	delay_ms(100);
	MS561101BA_RESET();	 // Reset Device  复位MS5611
	delay_ms(100);       //复位后延时（注意这个延时是一定必要的，可以缩短但似乎不能少于20ms）
	MS5611_init();	     //初始化MS5611
	 
	while(1)
	{
		delay_ms(200);
		
   	MS561101BA_GetTemperature();//获取温度
		MS561101BA_getPressure();   //获取大气压
		
		LCD_MS5611_DataShow();      //LCD显示粗处理的数据，显示
	}
}
