/*******************************************************************************
  * 文件名称: STM32F103 BMP280程序
	* 程序功能: 测气压、温度、高度
  * 程序作者：赵贵生
  * 版本编号：V1.0
  * 编写时间: 2018.05.13
  * 修改时间:
  ******************************************************************************
  * 修改记录:
  * 1、如果气压温度变化，加上IIR滤波，如果气压变化剧烈，去掉滤波。 2018.05.13
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	LED_Init();
	USART1_Init(9600);
	RS485_Init(9600);
	TFTLCD_Init();			//LCD初始化
	KEY_Init();
	bmp280Init();       //BMP280初始化
	
	FRONT_COLOR=BLACK;
	LCD_ShowString(20,50,200,16,16,"Puzhong STM32");
	LCD_ShowString(20,70,200,16,16,"PengZhuang IIC TEST");	
	LCD_ShowString(20,90,200,16,16,"BY ZhaoGuiSheng");
	LCD_ShowString(20,110,200,16,16,"2018/5/12");		
	 
 	FRONT_COLOR=BLUE;//设置字体为蓝色	  
	LCD_ShowString(20,150,200,16,16,"The temperature is    deg");	
	LCD_ShowString(20,170,200,16,16,"The airpress is");	
	LCD_ShowString(160,205,200,16,16,"pa");	  
	FRONT_COLOR=RED;  //设置字体为红色		
	
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
 //		h=high;                   //项目不需要使用高度值		
		   
		  LCD_ShowxNum(20+19*8,150,t,2,16,0);
		  LCD_ShowxNum(20+3*12,200,press1,6,24,0);
		  LCD_ShowxNum(20+10*12,200,press2,1,24,0);
		
			i++;
			if(i==20)
			{
				led1=!led1;//提示系统正在运行	
				i=0;
			}		   
	}
}
