#include "BMP180.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//气压传感器BMP180的STM32驱动，本程序只供学习使用 
//////////////////////////////////////////////////////////////////////////////////
  
//初始化IIC
void IIC_Init(void)
{
      
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
    
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); //PB10,PB11 输出高
 
}
//产生IIC起始信号
void IIC_Start(void)
{
 
SDA_OUT();     //sda线输出
IIC_SDA=1;   
IIC_SCL=1;
delay_us(4);
 IIC_SDA=0;//START:when CLK is high,DATA change form high to low
delay_us(4);
IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
 
} 
//产生IIC停止信号
void IIC_Stop(void)
{
 
SDA_OUT();//sda线输出
IIC_SCL=0;
IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 delay_us(4);
IIC_SCL=1;
IIC_SDA=1;//发送I2C总线结束信号
delay_us(4);  
 
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
 
u8 ucErrTime=0;
SDA_IN();      //SDA设置为输入 
IIC_SDA=1;delay_us(1);  
IIC_SCL=1;delay_us(1);
while(READ_SDA)
{
 
ucErrTime++;
if(ucErrTime>250)
{
 
IIC_Stop();
return 1;
 
}
 
}
IIC_SCL=0;//时钟输出0   
return 0; 
 
}
//产生ACK应答
void IIC_Ack(void)
{
 
IIC_SCL=0;
SDA_OUT();
IIC_SDA=0;
delay_us(2);
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
 
}
//不产生ACK应答   
void IIC_NAck(void)
{
 
IIC_SCL=0;
SDA_OUT();
IIC_SDA=1;
delay_us(2);
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
 
}     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答 
void IIC_Send_Byte(u8 txd)
{
                         
    u8 t;  
SDA_OUT();    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
               
        //IIC_SDA=(txd&0x80)>>7;
if((txd&0x80)>>7)
IIC_SDA=1;
else
IIC_SDA=0;
txd<<=1;  
delay_us(2);   //对TEA5767这三个延时都是必须的
IIC_SCL=1;
delay_us(2);
IIC_SCL=0;
delay_us(2);
     
}
 
}    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
u8 IIC_Read_Byte(unsigned char ack)
{
 
unsigned char i,receive=0;
SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
{
 
        IIC_SCL=0;
        delay_us(2);
IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;  
delay_us(1);
     
}
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK  
    return receive;
 
}
 
u8 BMP180_ReadOneByte(u8 ReadAddr)
{
   
u8 temp=0;      
    IIC_Start(); 
IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
IIC_Wait_Ack();  
    IIC_Send_Byte(ReadAddr);   //发送要读的寄存器地址
IIC_Wait_Ack();   
IIC_Start();     
IIC_Send_Byte(BMP180_RD_ADDR);           //进入接收模式  
IIC_Wait_Ack();
    temp=IIC_Read_Byte(0);  
    IIC_Stop();//产生一个停止条件   
return temp;
 
}
 
 
 
// u16 BMP180_ReadTowByte(u8 ReadAddr)
// {
   
// u16 temp=0;      
//     IIC_Start(); 
// IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
// IIC_Wait_Ack();  
//     IIC_Send_Byte(ReadAddr);   //发送要读的寄存器地址
// IIC_Wait_Ack();   
// IIC_Start();     
// IIC_Send_Byte(BMP180_RD_ADDR);           //进入接收模式  
// IIC_Wait_Ack();
// IIC_Send_Byte(BMP180_RD_ADDR);           //进入接收模式  
// IIC_Wait_Ack();
//     temp=IIC_Read_Byte(0);  
//     IIC_Stop();//产生一个停止条件   
// return temp;
//
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址   
//DataToWrite:要写入的数据
void BMP180_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
           
    IIC_Start(); 
 
IIC_Send_Byte(BMP180_WR_ADDR);    //发送写命令
  
IIC_Wait_Ack();  
    IIC_Send_Byte(WriteAddr);   //发送地址
IIC_Wait_Ack();      
IIC_Send_Byte(DataToWrite);     //发送字节  
IIC_Wait_Ack();        
    IIC_Stop();//产生一个停止条件
delay_us(1);
 
}
 
// int BMP180_CRC_Read(u8 addr)
// {
 
// u16 temp;
// temp = (BMP180_ReadOneByte(addr)<<8)|BMP180_ReadOneByte(addr+1);   
// return temp;
//
}
short BMP180_CRC_Read(u8 addr)
{
 
	u8 msb,lsb;
	short data;
	IIC_Start();
	IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
	IIC_Wait_Ack();  
	IIC_Send_Byte(addr);   //发送要读的寄存器地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(BMP180_RD_ADDR);
	 
	IIC_Wait_Ack();
	msb=IIC_Read_Byte(1);
	//IIC_Wait_Ack();
	lsb=IIC_Read_Byte(0);
	//IIC_Wait_Ack();
	IIC_Stop();
	delay_us(5);
	data= msb << 8;
	data|= lsb;
	return data;
 
}
u16 BMP085_Get_UT(void)
{
 
	IIC_Start();
	IIC_Send_Byte(BMP180_WR_ADDR);//发送写命令
	//IIC_Wait_Ack();
	IIC_Send_Byte(0xf4);        //发送字
	//IIC_Wait_Ack();
	IIC_Send_Byte(0x2E);
	IIC_Stop();
	delay_ms(5);
	return BMP180_CRC_Read(0xf6);
 
}
long  BMP_UP_Read(void)
{
 
	long temp;
	BMP180_WriteOneByte(0xF4,0x34+(oss<<6));
	delay_ms(27);
	temp=(BMP180_ReadOneByte(0xf6)<<16)|(BMP180_ReadOneByte(0xf7)<<8)|BMP180_ReadOneByte(0xf8);
	return temp>>(8-oss);
 
}
 
long BMP085_Get_UP(void)
{
 
	long pressure=0;
	u8 msb,lsb,xlsb;
	BMP180_WriteOneByte(0xF4,0x34+(oss<<6));
	delay_ms(2+(3<<oss));
	IIC_Start();
	IIC_Send_Byte(BMP180_RD_ADDR);
	IIC_Wait_Ack();
	msb=IIC_Read_Byte(1);
	lsb=IIC_Read_Byte(1);
	xlsb=IIC_Read_Byte(0);
	IIC_Stop();
	pressure=(msb<<16)|(lsb<<8)|xlsb;
	return pressure>>(8-oss);
 
}