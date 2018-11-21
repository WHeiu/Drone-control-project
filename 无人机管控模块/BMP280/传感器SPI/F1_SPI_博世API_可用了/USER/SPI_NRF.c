/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��SPI_NRF.c
 * ����    ��SPI_NRF24L01+����ģ��Ӧ�ú�����         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�-----------------------------|
 *          |                			  |
 *          |  	PA1            : NRF- CSN |
			|	PA5-SPI1-SCK   : NRF -SCK |
			|	PA6-SPI1-MISO  : NRF -MISO|
			|	PA7-SPI1-MOSI  : NRF -MOSI|
			|	PA3		   	   : NRF-IRQ  |
			|	PA2		   	   : NRF-CE   |
 *          |                			  |
 *           -----------------------------
 * ��汾  ��ST3.5.0
 *
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/
#include "SPI_NRF.h"
#include "usart1.h"


 u8 RX_BUF[RX_PLOAD_WIDTH];		//�������ݻ���
 u8 TX_BUF[TX_PLOAD_WIDTH];		//�������ݻ���
// u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};  // ����һ����̬���͵�ַ
// u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};



void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 


/*
 * ��������SPI_NRF_Init
 * ����  ��SPI�� I/O����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void SPI_BMP_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
 /*ʹ��GPIOB,GPIOD,���ù���ʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);

 /*ʹ��SPI1ʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//   /*����485оƬ�Ŀ�������GPIOA^0����ֹ����NRF*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);  
//  GPIO_ResetBits(GPIOA, GPIO_Pin_0);               //��ֹ485�ķ���ģʽ������ģʽ�Կ�

   /*���� SPI_NRF_SPI�� SCK,MISO,MOSI���ţ�GPIOA^5,GPIOA^6,GPIOA^7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù���
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  /*����SPI_NRF_SPI��CE���ţ�GPIOA^2��SPI_NRF_SPI�� CSN ����: NSS GPIOA^8*/
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


   /*����SPI_NRF_SPI��IRQ���ţ�GPIOA^3*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 
		  
  /* �����Զ���ĺ꣬��������csn���ţ�NRF�������״̬ */
  NRF_CSN_HIGH(); 
 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 					//��ģʽ
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	 				//���ݴ�С8λ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		 				//ʱ�Ӽ��ԣ�����ʱΪ��
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//��1��������Ч��������Ϊ����ʱ��
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		   					//NSS�ź����������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //8��Ƶ��9MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  				//��λ��ǰ
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}




/*
 * ��������SPI_NRF_RW
 * ����  ��������NRF��/дһ�ֽ�����
 * ����  ��д�������
 * ���  ����ȡ�õ�����
 * ����  ���ڲ�����
 */
u8 SPI_BMP_RW(u8 dat)
{  	
   /* �� SPI���ͻ������ǿ�ʱ�ȴ� */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
   /* ͨ�� SPI2����һ�ֽ����� */
  SPI_I2S_SendData(SPI1, dat);		
 
   /* ��SPI���ջ�����Ϊ��ʱ�ȴ� */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*
 * ��������SPI_NRF_WriteReg
 * ����  ��������NRF�ض��ļĴ���д������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   dat:��Ҫ��Ĵ���д�������
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ڲ�����
 */
u8 SPI_BMP_WriteReg(u8 reg,u8 dat)
{
 	u8 status;
//	 NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
    NRF_CSN_LOW();
				
	/*��������Ĵ����� */
	status = SPI_BMP_RW(reg);
		 
	 /*��Ĵ���д������*/
    SPI_BMP_RW(dat); 
	          
	/*CSN���ߣ����*/	   
  	NRF_CSN_HIGH();	
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
}


/*
 * ��������SPI_NRF_ReadReg
 * ����  �����ڴ�NRF�ض��ļĴ�����������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 * ���  ���Ĵ����е�����
 * ����  ���ڲ�����
 */



u8 SPI_BMP_ReadReg(u8 reg)
{
 	u8 reg_val;

//	NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
 	NRF_CSN_LOW();
				
  	 /*���ͼĴ�����*/
	SPI_BMP_RW(reg); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI_BMP_RW(NOP);
	            
   	/*CSN���ߣ����*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	


/*
 * ��������SPI_NRF_ReadBuf
 * ����  �����ڴ�NRF�ļĴ����ж���һ������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   pBuf�����ڴ洢���������ļĴ������ݵ����飬�ⲿ����
		   bytes: pBuf�����ݳ���	
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ⲿ����
 */
u16 SPI_BMP_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
 	u8 status, byte_cnt;

//	  NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();
		
	/*���ͼĴ�����*/		
	status = SPI_BMP_RW(reg); 

 	/*��ȡ����������*/
	 for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
	   pBuf[byte_cnt] = SPI_BMP_RW(NOP); //��NRF24L01��ȡ����  

	 /*CSN���ߣ����*/
	NRF_CSN_HIGH();	
		
 	return status;		//���ؼĴ���״ֵ̬
}



/*
 * ��������SPI_NRF_WriteBuf
 * ����  ��������NRF�ļĴ�����д��һ������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   pBuf���洢�˽�Ҫд��д�Ĵ������ݵ����飬�ⲿ����
		   bytes: pBuf�����ݳ���	
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ⲿ����
 */
u16 SPI_BMP_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes)
{
	 u8 status,byte_cnt;
	 NRF_CE_LOW();
   	 /*�õ�CSN��ʹ��SPI����*/
	 NRF_CSN_LOW();			

	 /*���ͼĴ�����*/	
  	 status = SPI_BMP_RW(reg); 
 	
  	  /*�򻺳���д������*/
	 for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
		SPI_BMP_RW(*pBuf++);	//д���ݵ������� 	 
	  	   
	/*CSN���ߣ����*/
	NRF_CSN_HIGH();			
  
  	return (status);	//����NRF24L01��״̬ 		
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
