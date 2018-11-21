/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  ：SPI_NRF.c
 * 描述    ：SPI_NRF24L01+无线模块应用函数库         
 * 实验平台：野火STM32开发板
 * 硬件连接：-----------------------------|
 *          |                			  |
 *          |  	PA1            : NRF- CSN |
			|	PA5-SPI1-SCK   : NRF -SCK |
			|	PA6-SPI1-MISO  : NRF -MISO|
			|	PA7-SPI1-MOSI  : NRF -MOSI|
			|	PA3		   	   : NRF-IRQ  |
			|	PA2		   	   : NRF-CE   |
 *          |                			  |
 *           -----------------------------
 * 库版本  ：ST3.5.0
 *
 * 库版本  ：ST3.5.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：http://www.amobbs.com/forum-1008-1.html
 * 淘宝    ：http://firestm32.taobao.com
**********************************************************************************/
#include "SPI_NRF.h"
#include "usart1.h"


 u8 RX_BUF[RX_PLOAD_WIDTH];		//接收数据缓存
 u8 TX_BUF[TX_PLOAD_WIDTH];		//发射数据缓存
// u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};  // 定义一个静态发送地址
// u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};



void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 


/*
 * 函数名：SPI_NRF_Init
 * 描述  ：SPI的 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_BMP_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
 /*使能GPIOB,GPIOD,复用功能时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);

 /*使能SPI1时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//   /*配置485芯片的控制引脚GPIOA^0，防止干扰NRF*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
//  GPIO_Init(GPIOA, &GPIO_InitStructure);  
//  GPIO_ResetBits(GPIOA, GPIO_Pin_0);               //禁止485的发送模式，接收模式仍开

   /*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚，GPIOA^5,GPIOA^6,GPIOA^7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  /*配置SPI_NRF_SPI的CE引脚，GPIOA^2和SPI_NRF_SPI的 CSN 引脚: NSS GPIOA^8*/
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


   /*配置SPI_NRF_SPI的IRQ引脚，GPIOA^3*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //上拉输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 
		  
  /* 这是自定义的宏，用于拉高csn引脚，NRF进入空闲状态 */
  NRF_CSN_HIGH(); 
 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 					//主模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	 				//数据大小8位
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		 				//时钟极性，空闲时为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//第1个边沿有效，上升沿为采样时刻
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		   					//NSS信号由软件产生
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //8分频，9MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  				//高位在前
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}




/*
 * 函数名：SPI_NRF_RW
 * 描述  ：用于向NRF读/写一字节数据
 * 输入  ：写入的数据
 * 输出  ：读取得的数据
 * 调用  ：内部调用
 */
u8 SPI_BMP_RW(u8 dat)
{  	
   /* 当 SPI发送缓冲器非空时等待 */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
   /* 通过 SPI2发送一字节数据 */
  SPI_I2S_SendData(SPI1, dat);		
 
   /* 当SPI接收缓冲器为空时等待 */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*
 * 函数名：SPI_NRF_WriteReg
 * 描述  ：用于向NRF特定的寄存器写入数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   dat:将要向寄存器写入的数据
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：内部调用
 */
u8 SPI_BMP_WriteReg(u8 reg,u8 dat)
{
 	u8 status;
//	 NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
    NRF_CSN_LOW();
				
	/*发送命令及寄存器号 */
	status = SPI_BMP_RW(reg);
		 
	 /*向寄存器写入数据*/
    SPI_BMP_RW(dat); 
	          
	/*CSN拉高，完成*/	   
  	NRF_CSN_HIGH();	
		
	/*返回状态寄存器的值*/
   	return(status);
}


/*
 * 函数名：SPI_NRF_ReadReg
 * 描述  ：用于从NRF特定的寄存器读出数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 * 输出  ：寄存器中的数据
 * 调用  ：内部调用
 */



u8 SPI_BMP_ReadReg(u8 reg)
{
 	u8 reg_val;

//	NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
 	NRF_CSN_LOW();
				
  	 /*发送寄存器号*/
	SPI_BMP_RW(reg); 

	 /*读取寄存器的值 */
	reg_val = SPI_BMP_RW(NOP);
	            
   	/*CSN拉高，完成*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	


/*
 * 函数名：SPI_NRF_ReadBuf
 * 描述  ：用于从NRF的寄存器中读出一串数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   pBuf：用于存储将被读出的寄存器数据的数组，外部定义
		   bytes: pBuf的数据长度	
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：外部调用
 */
u16 SPI_BMP_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
 	u8 status, byte_cnt;

//	  NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();
		
	/*发送寄存器号*/		
	status = SPI_BMP_RW(reg); 

 	/*读取缓冲区数据*/
	 for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
	   pBuf[byte_cnt] = SPI_BMP_RW(NOP); //从NRF24L01读取数据  

	 /*CSN拉高，完成*/
	NRF_CSN_HIGH();	
		
 	return status;		//返回寄存器状态值
}



/*
 * 函数名：SPI_NRF_WriteBuf
 * 描述  ：用于向NRF的寄存器中写入一串数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   pBuf：存储了将要写入写寄存器数据的数组，外部定义
		   bytes: pBuf的数据长度	
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：外部调用
 */
u16 SPI_BMP_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes)
{
	 u8 status,byte_cnt;
	 NRF_CE_LOW();
   	 /*置低CSN，使能SPI传输*/
	 NRF_CSN_LOW();			

	 /*发送寄存器号*/	
  	 status = SPI_BMP_RW(reg); 
 	
  	  /*向缓冲区写入数据*/
	 for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_BMP_RW(*pBuf++);	//写数据到缓冲区 	 
	  	   
	/*CSN拉高，完成*/
	NRF_CSN_HIGH();			
  
  	return (status);	//返回NRF24L01的状态 		
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
