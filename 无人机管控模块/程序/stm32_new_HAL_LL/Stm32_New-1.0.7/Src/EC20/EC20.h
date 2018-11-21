#ifndef __EC20_H
#define __EC20_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
//#include "board.h"
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"


#define EC20_PWRKEY_UP   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define EC20_PWRKEY_DOWN HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)

#define EC20_RST_UP      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
#define EC20_RST_DOWN    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)

#define EC20_DIS_UP      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define EC20_DIS_DOWN    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)

#define EC20_RDY_UP      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define EC20_RDY_DOWN    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

#define EC20_NET_MODE_UP      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define EC20_NET_MODE_DOWN    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern __IO ITStatus UartReady;
#define EC20_Sendbyte(x)       HAL_UART_Transmit(&huart4,&x,1,10)//USART3_SendByte                  //几个重要接口的定义
//#define EC20_USART_IRQHandler //USART3_IRQHandler
//#define EC20_Receivebyte      //USART_ReceiveData(USART3)
//#define EC20_USART            //USART3

typedef struct    
{
 unsigned char status;//?????
 unsigned char IMEI[20];//IMEI??
 unsigned char ipstatus;
 unsigned char tcpstaus;
 int sendcount;
 unsigned char datastatus;
 unsigned char ackstatus;
	unsigned char getcount;
	unsigned char getok;
	unsigned char tcpcount;
	unsigned char tcpflag;
	unsigned char enable;
	unsigned char dataunm;
}GSM_init;

typedef struct    
{

 unsigned char Realtime[25];//????
 unsigned char Qcell[25];//?????
 unsigned char GPS[30];//?????
 unsigned char sendplus[100];
unsigned char senddata[100];
unsigned char sendackdata[100];	
unsigned char sendgpsdata[100];	
unsigned char count;
unsigned char starnum[5];//卫星数量
unsigned char speed[10];//速度值
unsigned char getnum;
unsigned char hardfalut;
}GPS_DATA;

void EC20_send_data(char * data);
//uint8_t wait(char *str,uint32_t timeout);
void EC20_init();                          //手册推荐的初始化流程
void EC20_sendmsg(char *tell,char *data);  //发送短信
void EC20_call(char *tell);                //打电话
void tcp_connect_sever(char * ip,char *com);//TCP连接 连接上后用 EC20_send_data() 函数可以直接发数据

void EC20_start_up(void);
//void EC20_Sendbyte(char data) ;


void EC20_SendTest(char i);
void EC20_USART_SMIP_Interrupt(UART_HandleTypeDef *huart);

void EC20_UART_PROT_INIT(void);
void EC20_uart4_send(uint8_t* UART4_Tx);
void EC20_uart4_receive(uint8_t* tempdata);
void EC20_uart_SendAndReceive(uint8_t* UART4_Tx,uint8_t* tempdata);
void Get_IMEI(void);
void EC20_send_message(uint8_t* tempdata,uint8_t* UART4_Tx_MEGPhne,uint8_t* UART4_Tx_SEND);
void  EC20_GPS_Init(void);
void EC20Send_StrData(char *bufferdata);

extern int Uart_count;
extern void Delay(unsigned int xms);



#endif



