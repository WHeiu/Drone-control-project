#include "user.h"
#include "EC20.h"
#define RXLENN 200
extern char ReceiveBuff[];
extern char UART4_ReceiveBuff[];
extern 	int UART4_Receive_Count;
extern 	int Receive_Count;
char *sstrx;

void USART3_CLR_RecvBuf(void)
{
	memset(ReceiveBuff, 0, RXLENN);    //���USART3���ջ�����
	Receive_Count = 0;
}

 void UART4_CLR_RecvBuf(void)
{
	memset(UART4_ReceiveBuff, 0, RXLENN);    //���UART4���ջ�����
	UART4_Receive_Count=0;
}


void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen)
{
	int i = 0;
	for(i = 0; i < ucLen; i++)
	{
		//��ʽ����str,ÿunsigned char ת���ַ�ռ��λ��%xд��%Xд��
		sprintf(str + i * 2, "%02x", UnChar[i]);
	}

}
//���ڽ���������
void USART3_Ack_Interaction(void)
{
			
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"BMP280?");  //��ʾ��ѹ��BMP280����ֵ
		if(sstrx)
		{		
			BMP280_Get_Data();		  //��ȡ��ģ����ѹ��BMP280�ɼ�ֵ
			USART3_CLR_RecvBuf();
		}			
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"MPU9250?");   //��ʾ���ᴫ����MPU9250����
		if(sstrx)
		{		
			MPU9250_Get_Data();		//��ȡ��ģ����ᴫ����MPU9250�ɼ�����
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"EC20?");
		if(sstrx)
		{		
			EC20_Get_Data();		//��ȡ��ģ��EC20״̬		
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"Change_Server_IP");
		if(sstrx)
		{		
			EC20_Change_Server_IP();		//��ȡ��ģ��EC20״̬		
			USART3_CLR_RecvBuf();
		}
		

}

















