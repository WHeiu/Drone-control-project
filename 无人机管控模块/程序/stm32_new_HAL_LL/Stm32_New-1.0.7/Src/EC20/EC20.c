#include "EC20.h"
#include "main.h"
#include "usart.h"
GPS_DATA gspdata;
GSM_init GSMinit;

extern void Delay(unsigned int xms);
extern	uint8_t UART4_Tx_TEST1[];
extern	uint8_t UART4_Tx_TEST2[];
extern	uint8_t UART4_Tx_TEST3[];
extern	uint8_t UART4_Tx_TEST4[];
extern	uint8_t UART4_Tx_TEST5[];
	
	/*�绰���ɱ䣩*/
extern	uint8_t UART4_Tx_TEST6[];
extern	uint8_t UART4_Tx_TEST7[];
	
extern	uint8_t UART4_Tx_GPS_TEST0[];
extern	uint8_t UART4_Tx_GPS_TEST1[];
extern	uint8_t UART4_Tx_GPS_TEST2[];
extern	uint8_t UART4_Tx_GPS_TEST3[];
extern	uint8_t UART4_Tx_GPS_TEST4[];
extern	uint8_t UART4_Tx_GPS_TEST5[];

	/*IP ��ַ�ڣ��ɱ䣩*/
extern  uint8_t UART4_Tx_TCP_TEST1[];
extern	uint8_t UART4_Tx_TCP_TEST2[];
extern	uint8_t UART4_Tx_TCP_TEST3[];
	
extern	uint8_t UART4_Tx_MESSAGE1[];
extern	uint8_t UART4_Tx_MESSAGE2[];
extern	uint8_t UART4_Tx_MESSAGE3[];
extern	uint8_t UART4_Tx_MESSAGE4[];

extern char ReceiveBuff[];
extern char UART4_ReceiveBuff[];
char EC20_rx_buff[512];
uint8_t EC20_rx_flag=0;
uint8_t EC20_rx_num=0;
uint8_t EC20_rx_tempcount=0;

char *gps_strx,*gps_extstrx,*gps_Readystrx;
char *EC20_init_cmd[]={
    	//"AT+QURCCFG=\"urcport\",\"uart1\"\r\n",
	      "AT+QURCCFG=?\r\n",
	      "ATI\r\n",
      	"ATE1\r\n",
      	"AT+CPIN?\r\n",
      	"AT+CSQ\r\n",
      	"AT+CGMM\r\n",
      	"AT+CGMR\r\n",
	      "AT+CGSN\r\n"
};

char *EC20_sendmsg_cmd[]={
      	"AT+CPMS?\r\n",
      	"AT+CNMI=1\r\n",
      	"AT+CSCA?\r\n",
      	"AT+CMGF=1\r\n",
};


void EC20_start_up()
{

      	EC20_RDY_DOWN;
        EC20_DIS_UP;
	      EC20_RST_UP;
       	EC20_NET_MODE_DOWN;
      	EC20_PWRKEY_DOWN;
	      HAL_Delay(500);

}
#if 0
void EC20_Sendbyte(char x)       
{
	      printf("EC20_Sendbyte \n");
      	HAL_UART_Transmit_IT(&huart4,&x,1);
	      //while (UartReady != SET);

	      printf("EC20_Sendbyte    Finsh\n");
  
}

void EC20_Recvbyte(x)       
{
	     HAL_UART_Receive_IT(&huart4,&x,1);
	     while (UartReady != SET);
}
#endif
void EC20_send_data(char * UART4_Tx)
{
	
	    int size=strlen((const char*)UART4_Tx);
    	if(HAL_UART_Transmit_IT(&huart4, (uint8_t*)UART4_Tx, size)!= HAL_OK)
      {
		    Error_Handler();  //SEND "ATI\r\n"
      }
     	UartReady=RESET;
}

uint8_t wait(char *str,uint32_t timeout)    
{
	    uint16_t i=0,j=200;
    	uint32_t count=0;
    	uint8_t statue=0;
    	while(EC20_rx_flag==0)
    	{
	    	count++;
		    if(count>timeout)
	      	{
	      		statue=1;
		      	count=0;
		      	break;
		      }
      }
	
    	while(strstr(EC20_rx_buff,str)==NULL)
    	{
		    count++;
		    if(count>timeout)
	    	  {
		      	statue=1;
		      	count=0;
		      	break;
	      	}
	    }
    	EC20_rx_flag=0;
	    for(i=0;i<j;i++)
	    {
		      EC20_rx_buff[i]=0;
    	}
	    EC20_rx_num=0;
      EC20_rx_tempcount=0;
	
    	return statue;
}
void EC20_SendTest(char i)
{

    	EC20_send_data(EC20_init_cmd[i]);
	
}

void EC20_init()
{
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
		do
			EC20_send_data(EC20_init_cmd[i]);
		while(wait("OK",0xffffff));
	}
}

void EC20_sendmsg(char *tell,char *data)
{
	uint8_t i=0,temp=0;
	for(i=0;i<4;i++)
	{
		do
			EC20_send_data(EC20_sendmsg_cmd[i]);
		while(wait("OK",0xffffff));
	}


	EC20_send_data("AT+CMGS=\"");
	EC20_send_data(tell);
	EC20_send_data("\"\r\n");
	wait("> ",0xffffff);

	EC20_send_data(data);

	temp=0x1a;
	EC20_Sendbyte(temp);
	
	wait("OK",0xffffff);
}

void EC20_call(char *tell)
{
	EC20_send_data("ATD");
	EC20_send_data(tell);
	EC20_send_data(";\r\n");
	wait("OK",0xffffff);
}

void tcp_connect_sever(char * ip,char *com)
{
	EC20_send_data("AT+QIOPEN=1,0,\"TCP\",\"");
	EC20_send_data(ip);
	EC20_send_data("\",");
	EC20_send_data(com);
	EC20_send_data(",0,2\r\n");
	wait("CONNECT",0xffffff);
}

/*************/
//Ŀǰ��Ϊ�����ã�ʵ��ʹ����Ҫ�ĳ����溯���ķ�ʽ
//���������Ѵ��ڵ����ݽ��д�ӡ���������ڶԱ�
void EC20_USART_SMIP_Interrupt(UART_HandleTypeDef *huart)
{
	uint8_t rbyte;

	// Check that the Read Data Register Not Empty interrupt is enabled and has occurred
	if((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET))
	{
		// Get byte from Read Data Register
		rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t)0xff);
		// Flush Read Data Register to prevent overflow
		__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
		// Push byte to HDLC layer
		EC20_rx_buff[EC20_rx_tempcount] = rbyte;   //��������
		EC20_rx_tempcount++;

		EC20_rx_flag=1;
	}
}


/*
void EC20_USART_IRQHandler ( void )
{	
	uint8_t ch;
	uint16_t i;

	if(USART_GetITStatus(EC20_USART, USART_IT_RXNE) != RESET)
	{ 
		if(EC20_rx_flag==0)
		{
			EC20_rx_buff[EC20_rx_tempcount] = EC20_Receivebyte;   //��������
			//USART_SendData(USART1,EC20_rx_buff[EC20_rx_tempcount]);
			EC20_rx_tempcount++;
		}
		else
			EC20_Receivebyte;
		
		if((strstr(EC20_rx_buff,"OK\r\n")!=NULL)||(strstr(EC20_rx_buff,"> ")!=NULL)||(strstr(EC20_rx_buff,"CONNECT\r\n")!=NULL))
		{
			EC20_rx_flag=1;
			EC20_rx_num=EC20_rx_tempcount;
			EC20_rx_tempcount=0;
			//printf("\nget\n");
		}
	} 
}
*/


void EC20_UART_PROT_INIT()
{

	uint8_t UART4_Tx_data[30]="AT+QURCCFG=\"urcport\",\"uart1\"\r\n";//¸Ä±äEC20 µÄ´®¿ÚÄ£Ê½
	if(HAL_UART_Transmit_IT(&huart4, (uint8_t*)UART4_Tx_data, (COUNTOF(UART4_Tx_data) - 1))!= HAL_OK)
	{
	Error_Handler();  
	}
	while (UartReady != SET)
	{
	}
	UartReady=RESET;
}

void EC20_uart4_send(uint8_t* UART4_Tx)
{
	int size=strlen((const char*)UART4_Tx);
	if(HAL_UART_Transmit(&huart4, (uint8_t*)UART4_Tx, size,100)!= HAL_OK)
	{
		Error_Handler();  //SEND "ATI\r\n"
	}
	UartReady=RESET;
}	

void EC20_uart4_receive(uint8_t* tempdata)
{			
	int uartdata_size=60;

	if(HAL_UART_Receive(&huart4, (uint8_t *)tempdata, uartdata_size,60) != HAL_OK)
	{
		Error_Handler();	//receive data from uart4 to tempdata
	}

	UartReady=RESET;

	Uart_count++;
	Delay(500);

	if(HAL_UART_Transmit(&huart3, (uint8_t*)tempdata, uartdata_size,60)!= HAL_OK)
	{
		Error_Handler();  	//transmit tempdata to uart3
	}

	UartReady=RESET;
}

void EC20_uart_SendAndReceive(uint8_t* UART4_Tx,uint8_t* tempdata)
{
	int size=strlen((const char*)UART4_Tx);
	int uartdata_size=60;
	
	if(HAL_UART_Transmit(&huart4, (uint8_t*)UART4_Tx, size,100)!= HAL_OK)
	{
		Error_Handler();  //SEND "AT"command
	}
	UartReady=RESET;
	

	if(HAL_UART_Receive(&huart4, (uint8_t *)tempdata, uartdata_size,60) != HAL_OK)
	{
		Error_Handler();	//receive data from uart4 to tempdata
	}
	UartReady=RESET;
	Uart_count++;

	Delay(500);

	if(HAL_UART_Transmit(&huart3, (uint8_t*)tempdata, uartdata_size,100)!= HAL_OK)
	{
		Error_Handler();  	//transmit tempdata to uart3
	}

	UartReady=RESET;
}


void EC20_send_message(uint8_t* tempdata,uint8_t* UART4_Tx_MEGPhne,uint8_t* UART4_Tx_SEND)
{
			printf("READY TO SEND~~~ \n");
		
		  uint8_t count_ex=100;
	    uint8_t count_OK=0;
	  	uint8_t m=0;
		  uint8_t *pTempdate;
		
		  uint8_t UART4_Tx_CMGS_end[]={0x1A};
		
		/************init the environment EC20 of message processing **************/
		for( m=0;m<=3;m++)
		{	
			count_ex=100;
			do
			{
				EC20_uart_SendAndReceive((uint8_t *)EC20_sendmsg_cmd[m],tempdata);//send init_data
				pTempdate=tempdata;
				pTempdate++;
				count_OK=0;
				while(((*pTempdate)!='K') && ((*(pTempdate-1))!='O'))//if receive the right data'OK',end send
				{
					pTempdate++;
					count_OK++;
					if(count_OK==100)
					{	
						pTempdate=tempdata;
						break;
					}
				}	
				/*******check the data ************/
				if(((*pTempdate)=='K') && ((*(pTempdate-1))=='O'))
				{
					Delay(2000);
					printf("init %% %d \n",(int)m);
					break;
				}
				count_ex--;
				printf("init fall..\n");
			}while(count_ex!=0);
		}
		
		
		Delay(2000);
		printf("INIT OK \n*******************************\n*****************************\n");
		
		/*********send AT+CMGS=" YOUR PHONE"*******************/
		EC20_uart_SendAndReceive((uint8_t *)UART4_Tx_MEGPhne,tempdata);
		Delay(50);
		
		EC20_uart4_receive(tempdata);
		pTempdate=tempdata;
		
		while((*pTempdate)!='>')
		{
			EC20_uart4_receive(tempdata);
			pTempdate++;
			count_OK++;
			if(count_OK==100)
			{	
				count_OK=0;
				break;
			}
		}//if receive the right data'OK',end send
		
		
		/*******send the content to the EC20*********/
		if((*pTempdate)!='>')
		{
			EC20_uart4_send(UART4_Tx_SEND);
			EC20_uart4_send(UART4_Tx_CMGS_end);
			Delay(2000);
		}
    	else if((*pTempdate)=='>')
		{
			EC20_uart4_send(UART4_Tx_SEND);
			EC20_uart4_send(UART4_Tx_CMGS_end);
			Delay(2000);
			
		}
	printf("Finished send message \n");
}

void Get_IMEI(void)
{
    char i=0,*strx;
    //   printf("AT\r\n"); //ͬ��EC20����
	  //EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0);
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT \r\n",strlen("AT\r\n"));
    LL_mDelay(300);
    strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    while(strx==NULL)
    {
  	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
       LL_mDelay(300);
        strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }	
 		 printff(USART3, "Get_IMEI_START_1\r\n",strlen("Get_IMEI_START_1\r\n"));
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		USART_CLR_RecvBuf();
	  printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	   LL_mDelay(300);
  while(1)
    {
		      if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//��ȡģ��IMEI��ֵ
							 	         break;
			      else
        {
         		USART_CLR_RecvBuf();
            printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
					            Delay(300);		 
        }
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		//printff(USART3, "Get_IMEI_START_2\r\n",strlen("Get_IMEI_START_2\r\n"));
  for(i=0;i<15;i++)
    {
        GSMinit. IMEI[i]=UART4_ReceiveBuff[i+2];
        GSMinit.IMEI[15]=',';
    }	
  for(i=0;i<16;i++)
	gspdata.senddata[i]=GSMinit.IMEI[i];//��ȡ��IMEIֵ
   // printf("AT+QGPS=1\r\n");//��GPS��Դ
  printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
   LL_mDelay(300);	
	//printf("AT+QICLOSE=0\r\n");//�ر���һ������
	 printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
   LL_mDelay(300);	
}

/*  ��ȡGPS�ϱ������� EC20��ʼ��*/
void  EC20_GPS_Init(void)
{
  //  printf("AT\r\n"); 
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
  while(gps_strx==NULL)
    {
        USART_CLR_RecvBuf();	
    //    printf("AT\r\n"); 
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
			  printff(UART4, "AT\r\n",strlen("AT\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();	
  //  printf("AT+QGPS?\r\n");//��ѯ��ǰ״̬
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//�����Ѿ��ϵ�
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
  if(gps_strx==NULL)//���û�ϵ���ϵ磬�ϵ�Ͳ�Ҫ�ظ��ϵ�
   // printf("AT+QGPS=1\r\n");//��GNSS�ϵ�
	{
	  printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
	  printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
	}
    LL_mDelay(500);
 //   printf("ATE0\r\n"); //�رջ���
	  printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
    LL_mDelay(500);
	printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();	
 //   printf("AT+CSQ\r\n"); //���CSQ
	 printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
   	printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
    LL_mDelay(500);
 //   printf("ATI\r\n"); //���ģ��İ汾��
	  printff(USART3, "ATI\r\n",strlen("ATI\r\n"));
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
    LL_mDelay(500);
    /////////////////////////////////
   // printf("AT+CPIN?\r\n");//���SIM���Ƿ���λ
	  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//�鿴�Ƿ񷵻�ready
  while(gps_strx==NULL)
    {
        USART_CLR_RecvBuf();
   //     printf("AT+CPIN?\r\n");
		  	printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		  	printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//���SIM���Ƿ���λ���ȴ�����λ�������ʶ�𲻵���ʣ��Ĺ�����û������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();	
    ///////////////////////////////////
 //   printf("AT+CREG?\r\n");//�鿴�Ƿ�ע��GSM����
		printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
		printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
  while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        USART_CLR_RecvBuf();
      //  printf("AT+CREG?\r\n");//�鿴�Ƿ�ע��GSM����
			  printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
			  printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
    /////////////////////////////////////
 //   printf("AT+CGREG?\r\n");//�鿴�Ƿ�ע��GPRS����
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
  while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        USART_CLR_RecvBuf();
      // printf("AT+CGREG?\r\n");//�鿴�Ƿ�ע��GPRS����
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		   	printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
   // printf("AT+COPS?\r\n");//�鿴ע�ᵽ�ĸ���Ӫ�̣�֧���ƶ� ��ͨ ���� 
		printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
   // printf("AT+QICLOSE=0\r\n");//�ر�socket����
		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
		printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
 //   printf("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n");//����APN�����û���������
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
 //   printf("AT+QIDEACT=1\r\n");//ȥ����
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
		printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
  //  printf("AT+QIACT=1\r\n");//����
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
		printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
   // printf("AT+QIACT?\r\n");//��ȡ��ǰ����IP��ַ
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
		printff(UART4, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
 //  printf("AT+QIOPEN=1,0,\042TCP\042,\042103.46.128.49\042,28180,0,1\r\n");//��������Ҫ��½��IP���룬����ֱ���³�ģʽ
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042103.46.128.49\042,28180,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042103.46.128.49\042,28180,0,1\r\n"));
//		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042103.46.128.49\042,28180,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042103.46.128.49\042,28180,0,1\r\n"));	
		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
  while(gps_strx==NULL)
    {
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
        LL_mDelay(100);
       
    }
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
}		

///�����ַ�������
void EC20Send_StrData(char *bufferdata)
{
    uint8_t untildata=0xff;
  //  printf("AT+QISEND=0\r\n");
	  printff(USART3, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
		printff(UART4, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
    LL_mDelay(100);
   // printf(bufferdata);
	  printff(USART3, bufferdata,strlen(bufferdata));
		printff(UART4, bufferdata,strlen(bufferdata));
    LL_mDelay(100);	
 //   USART_SendData(USART2, (uint8_t) 0x1a);//������ɺ���
	  LL_USART_TransmitData8(UART4, (uint8_t) 0x1a);
	printff(USART3, "11111\r\n",strlen("11111\r\n"));
  while(LL_USART_IsActiveFlag_TC(UART4) == RESET)
	{
	}
	printff(USART3, "222222\r\n",strlen("222222\r\n"));
	LL_mDelay(100);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
  printff(USART3, "3333333333\r\n",strlen("3333333333\r\n"));
	 printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
	while(gps_strx==NULL)
    {
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
        LL_mDelay(10);
    }
		 printff(USART3, "444444444444\r\n",strlen("444444444444\r\n"));
    LL_mDelay(100);
	  printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    USART_CLR_RecvBuf();
    //printf("AT+QISEND=0,0\r\n");
	  printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
    LL_mDelay(200);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
   printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		while(untildata)
    {
       // printf("AT+QISEND=0,0\r\n");
				printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		    printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
        LL_mDelay(200);
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
        gps_strx=strstr((char*)gps_strx,(char*)",");//��ȡ��һ��,
        gps_strx=strstr((char*)(gps_strx+1),(char*)",");//��ȡ�ڶ���,
        untildata=*(gps_strx+1)-0x30;
        USART_CLR_RecvBuf();
    }
    printff(USART3, "Send Flish!!!!!\r\n",strlen("Send Flish!!!!!\r\n"));
}


