#include "IS25LPXX.h" 

//QUADSPI driver. Date:2018/8/26
//Written by Hunter

__IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch, TimeOut;

#define QSPI		hqspi

QSPI_CommandTypeDef s_command;
uint8_t QSPI_mode = FALSE;
uint8_t Addr4_mode = FALSE;

//Read IS25LPXX Status Register
//BIT7  6   5   4   3   2   1   0
//SRWD  QE  BP3 BP2 BP1 BP0 WEL WIP, R/W
//SRWD: SR write disable, R/W
//QE:   Quad Enable bit, R/W
//BPX:  Block Protection, R/W
//WEL:  Write Enable Latch, R/W
//WIP:  Write In Progress Bit, Ready Only, '1' BUSY, '0' READY
//Defaut: 0x00
//Instruction: IS25LPXX_ReadStatusReg
uint8_t IS25LPXX_ReadSR(uint8_t SRcmd)   
{  
	uint8_t byte=0;
	printf("\r\nEnter IS25LPXX_ReadSR\r\n");
	s_command.Instruction       = SRcmd;
	s_command.Address           = 0x0;
	s_command.AlternateBytes    = 0x0;
	if (Addr4_mode==1)
		s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
		s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.AlternateBytesSize  = QSPI_ALTERNATE_BYTES_8_BITS;
	s_command.DummyCycles       = 0;
	if (QSPI_mode==1)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	if (QSPI_mode==1)
		s_command.DataMode          = QSPI_DATA_4_LINES;
	else
		s_command.DataMode          = QSPI_DATA_1_LINE;
	s_command.NbData            = 1;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
	{
		printf("\r\nWarning: Failed to Operate 'Read FLASH Status Register' command cause of BUSY!\r\n");
		//TO DO: return?
	}
	if (HAL_QSPI_Receive(&QSPI,&byte,0xffff) != HAL_OK)
	{
		printf("\r\nError: Failed to read FLASH Status Register\r\n");
		//TO DO: return?
	}
	printf("\r\nThe status register vaule of FLASH is: %X\r\n",byte);
	printf("\r\nExit IS25LPXX_ReadSR\r\n");
  return byte;
} 

//Write IS25LPXX Status Register
//ONLY SRWD,QE,BP3,BP2,BP1,BP0,WEL(bit 7 to bit 1) are R/W!!!
void IS25LPXX_Write_SR(uint8_t *srdata, uint8_t NumByteToWrite)   
{   
 	printf("\r\nEnter IS25LPXX_Write_SR\r\n");
	IS25LPXX_Write_Enable();                  //SET WEL 
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_WriteStatusReg;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.Address           = 0;
	if (Addr4_mode)
		s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
		s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	if (QSPI_mode)
		s_command.DataMode          = QSPI_DATA_4_LINES;
	else
		s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = NumByteToWrite;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Write Enable (Set WEL bit in Status Register)' command cause of BUSY!\r\n");
		//TO DO: return?
  }
  if (HAL_QSPI_Transmit(&QSPI,srdata,0xffff) != HAL_OK)
  {
  		printf("\r\nError: Failed to write data into Status Register\n");
		//TO DO: return?
  }
	IS25LPXX_Wait_Busy();						//Waiting for the completion of operation.  
	printf("\r\nExit IS25LPXX_Write_SR\r\n");

}   

//Enable Quad SPI mode operation --  QSPI
void IS25LPXX_Quad_Enable(void)
{
	uint8_t temp;
	//in SPI mode
 	printf("\r\nEnter IS25LPXX_Quad_Enable\r\n");
  s_command.Instruction       = IS25LPXX_EnterQPIMode;
  s_command.Address           = 0;
	s_command.AlternateBytes    = 0;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
	s_command.AlternateBytesSize= 0;
  s_command.DummyCycles       = 0;
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode          = QSPI_DATA_NONE;
  s_command.NbData            = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Set QSPI mode' command cause of BUSY!\r\n");
		//TO DO: return?
  }
	temp = IS25LPXX_ReadSR(IS25LPXX_ReadStatusReg);
	printf("After set QSPI mode, The status register value of FLASH is:%X",temp);
	if((temp&0x40) != 0x40)
	{		//Failed to set QSPI mode
		printf("\r\nError: Failed to Operate 'Set QSPI mode' command!\r\n");
		//TO DO: return?
		QSPI_mode = FALSE;
	}
	else
	{
		printf("\r\nEnable QSPI successfully!\r\n");
		QSPI_mode = TRUE;
		Addr4_mode = TRUE;
	}
 	printf("\r\nExit IS25LPXX_Quad_Enable\r\n");
}

//Set IS25LPXX write enable 	
//Set WEL bit (bit 1 in status register)
void IS25LPXX_Write_Enable(void)   
{
 	printf("\r\nEnter IS25LPXX_Write_Enable\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_WriteEnable;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Write Enable (Set WEL bit)' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }	      
 	printf("\r\nExit IS25LPXX_Write_Enable\r\n");
} 
//Reset IS25LPXX Write Enable
//Clear WEL bit (bit 1 in status register)
void IS25LPXX_Write_Disable(void)   
{  
 	printf("\r\nEnter IS25LPXX_Write_Disable\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_WriteDisable;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Reset Write Enable (clear WEL bit)' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }	      
 	printf("\r\nExit IS25LPXX_Write_Disable\r\n");
} 		

//Waiting for the finishment of writting operation
void IS25LPXX_Wait_Busy(void)   
{   
	while((IS25LPXX_ReadSR(IS25LPXX_ReadStatusReg)&0x01)==0x01);   // wait till WIP(BUSY) is cleared.
}  
	
//enable memory mapping
void IS25LPXX_Memory_Mapped_Enable(void)   
{
	QSPI_MemoryMappedTypeDef sMemMappedCfg;
	
 	printf("\r\nEnter IS25LPXX_Memory_Mapped_Enable\r\n");
	sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	if (Addr4_mode)
		s_command.Instruction       = IS25LPXX_FastReadQuadIO4Addr;
	else
		s_command.Instruction       = IS25LPXX_FastReadQIO3Addr;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.Address           = 0;
	if (Addr4_mode)
		s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
		s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE; //without AXh, means do not use AX mode
//	s_command.AlternateBytes 		= 0x00;
//	s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 6;
  s_command.NbData            = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	
	if (HAL_QSPI_MemoryMapped(&QSPI, &s_command, &sMemMappedCfg) != HAL_OK)
	{
		printf("\r\nWarning: Failed to Operate 'Memory mapping mode' command cause of BUSY!\r\n");
		//TO DO: return false?		
	} 
 	printf("\r\nExit IS25LPXX_Memory_Mapped_Enable\r\n");
} 

//chip erase
//waiting for a long time......
void IS25LPXX_Erase_Chip(void)   
{                                   
 	printf("\r\nEnter IS25LPXX_Erase_Chip\r\n");
  IS25LPXX_Write_Enable();                  //SET WEL 
  IS25LPXX_Wait_Busy();  										//Waiting for the completion of operation.
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_ChipErase;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.Address           = 0;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Erase whole chip' command cause of BUSY!\r\n");
		//TO DO: return false?		
  } 
	IS25LPXX_Wait_Busy();   				   	//Waiting for the completion of operation.
 	printf("\r\nExit IS25LPXX_Erase_Chip\r\n");
}   
//Erase a sector
//Dst_Addr:the address of sector, plase set it according to the chip capacits.
//The min time to erase a sector is:150ms
//The sector address range of IS25LP128 is 0 to 4095.
void IS25LPXX_Erase_Sector(uint32_t Dst_Addr)   
{    
 	printf("\r\nEnter IS25LPXX_Erase_Sector\r\n");
 	if (Dst_Addr>4095)
	{
		printf("\r\nError: the parameter is error when calling sector erase cmd!\n");
		return;
	}
	Dst_Addr*=4096;
  IS25LPXX_Write_Enable();                  //SET WEL 	 
  IS25LPXX_Wait_Busy();   									//Waiting for the completion of operation.
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  if (Addr4_mode)
	  s_command.Instruction       = IS25LPXX_SectroEraseWith4Addr;
	else
	  s_command.Instruction       = IS25LPXX_SectorErase3Addr;
	if (QSPI_mode)
    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
	else
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.Address           = Dst_Addr;
	if (Addr4_mode)
		s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
		s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Erase sector %d' command cause of BUSY!\r\n",Dst_Addr);
		//TO DO: return false?		
  }
  IS25LPXX_Wait_Busy();   				   	//Waiting for the completion of operation.
 	printf("\r\nExit IS25LPXX_Erase_Sector\r\n");
}
//Read chip ID
//return:				   
//0X9D17,chip is IS25LP128 	  
uint16_t IS25LPXX_ReadID(uint8_t MDFirst)
{
	uint8_t Temp[2];	
//	uint8_t *p;
//	uint32_t RxXferCount=0;
 	printf("\r\nEnter IS25LPXX_ReadID\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_ReadMIDDID;
	if (QSPI_mode)
    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  else
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
	if(MDFirst)	
    s_command.Address           = 0;
	else
    s_command.Address           = 1;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.AlternateBytes 		= 0x00;
	s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	if (QSPI_mode)
    s_command.DataMode          = QSPI_DATA_4_LINES;
  else
    s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Read chip MID and DID' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }
	RxCplt=0;
	if (HAL_QSPI_Receive_DMA(&QSPI,Temp) != HAL_OK)
  {
		printf("\r\nError: Failed to Read chip MID and DID!\n");
		//TO DO: return false?		
  }	
	while(RxCplt == 0);	
	
 	printf("\r\nExit IS25LPXX_ReadID\r\n");
	return (Temp[0]<<8) + Temp[1];
} 


void IS25LPXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{   	
 	printf("\r\nEnter IS25LPXX_Read\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  if (Addr4_mode)
	  s_command.Instruction       = IS25LPXX_FastReadQuadIO4Addr;
	else
	  s_command.Instruction       = IS25LPXX_FastReadQIO3Addr;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.Address           = ReadAddr;
	if (Addr4_mode)
		s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
		s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	s_command.AlternateBytes 		= 0x00;
	s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 4;
  s_command.NbData            = NumByteToRead;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
		
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Fast Read Data' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }
  RxCplt=0;
	if (HAL_QSPI_Receive_DMA(&QSPI,pBuffer) != HAL_OK)
  {
		printf("\r\nError: Failed to Read Data!\r\n");
		//TO DO: return false?		
  }	
	while(RxCplt == 0);	
	IS25LPXX_Wait_Busy();
 	printf("\r\nExit IS25LPXX_Read\r\n");
} 


//In SPI mode, write less than 255 bytes into sepecial page (64KB).
//pBuffer: pointer to data buffer
//WriteAddr:the beginning addres (24bit)
//NumByteToWrite: number of data, it should be less than 256, and do not exceed the rest of sector. 
void IS25LPXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{  
 	printf("\r\nEnter IS25LPXX_Write_Page\r\n");
  IS25LPXX_Write_Enable();                  //SET WEL 
	
	if(QSPI_mode)
	  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
	  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  if (Addr4_mode)
    s_command.Instruction       = IS25LPXX_InputPageProgramWith4Addr;
  else
    s_command.Instruction       = IS25LPXX_InputPageProgram3Addr;
	if(QSPI_mode)
	  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
	else
	  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.Address           = WriteAddr;
  if (Addr4_mode)
    s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
	else
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	if(QSPI_mode)
    s_command.DataMode          = QSPI_DATA_4_LINES;
	else
    s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = NumByteToWrite;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&QSPI, &s_command,0xffff) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Write data into special page in Quad Data' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }
	TxCplt=0;
	if (HAL_QSPI_Transmit_DMA(&QSPI,pBuffer) != HAL_OK)
  {
		printf("\r\nError: Failed to Read Data!\r\n");
		//TO DO: return false?		
  }
	while(TxCplt == 0);	
	IS25LPXX_Wait_Busy();	
 	printf("\r\nExit IS25LPXX_Write_Page\r\n");

} 

//Write data into FLASH without check in SPI mode.
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void IS25LPXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
 	printf("\r\nEnter IS25LPXX_Write_NoCheck\r\n");
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		IS25LPXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
 	printf("\r\nExit IS25LPXX_Write_NoCheck\r\n");
} 

//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(32bit)						
//NumByteToWrite:要写入的字节数(最大65535) 
uint8_t IS25LPXX_BUFFER[4096];		 
void IS25LPXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;
 	__IO uint16_t i;    
	uint8_t * IS25LPXX_BUF;	
	uint8_t checksum;
  IS25LPXX_BUF=IS25LPXX_BUFFER; 
    
 	printf("\r\nEnter IS25LPXX_Write\r\n");
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		HAL_Delay(500);
		IS25LPXX_Read(IS25LPXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		printf("\r\nThe Flash content in sector %X is:\r\n",secpos);
		checksum = 255;
		for(i=0;i<secremain;i++) 
		{
			printf("%X ",IS25LPXX_BUF[i]);
			checksum &= IS25LPXX_BUF[i];
		}
		printf("\r\n");
		for(i=0;i<secremain;i++)//校验数据
		{
			if(IS25LPXX_BUF[secoff+i]!=pBuffer[i])break;//验证数据是否需要擦除和写  	  
		}		
		if((i<secremain) && (checksum != 255))//需要擦除
		{
			printf("\r\nErased sector address:%X, Number of data to be written:%d\r\n",WriteAddr,NumByteToWrite);//测试用
			IS25LPXX_Erase_Sector(secpos);	//擦除这个扇区
		}
		for(i=0;i<secremain;i++)	  	//复制
		{
			IS25LPXX_BUF[i+secoff]=pBuffer[i];	  
		}
		IS25LPXX_Write_NoCheck(IS25LPXX_BUF,secpos*4096,NumByteToWrite);//写入整个扇区  
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		  pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		  NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	}	
 	printf("\r\nExit IS25LPXX_Write\r\n");
}

//进入掉电模式
void IS25LPXX_PowerDown(void)   
{ 
 	printf("\r\nEnter IS25LPXX_PowerDown\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_DeepPowerDown;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&QSPI, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Enter Deep Power down' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }	    	      
  HAL_Delay(1);          //等待TPD  
 	printf("\r\nExit IS25LPXX_PowerDown\r\n");
}   

//唤醒
void IS25LPXX_WAKEUP(void)   
{  
 	printf("\r\nEnter IS25LPXX_WAKEUP\r\n");
	if (QSPI_mode)
		s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
	else
		s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = IS25LPXX_ReadIDReleasePD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&QSPI, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
		printf("\r\nWarning: Failed to Operate 'Wake up FLASH' command cause of BUSY!\r\n");
		//TO DO: return false?		
  }	    	      
  HAL_Delay(1);          //等待TRES1
 	printf("\r\nExit IS25LPXX_WAKEUP\r\n");
}   

void IS25LPXX_Init(void)
{ 
	uint16_t temp=0;
	uint8_t readid = 1;

 	printf("\r\nEnter IS25LPXX_Init\r\n");
	temp=IS25LPXX_ReadID(readid);
	while(temp !=IS25LP128)
	{
		printf("\r\nFailed to read FLASH ID. Waiting for 500ms to re-read!\r\n");
		if(readid) readid = 0;
		else readid = 1;
		HAL_Delay(1000);
		temp=IS25LPXX_ReadID(readid);
		printf("\r\nFLASH id is: %X\r\n",temp);
	}
	printf("\r\nFlash MID/DID is %X\r\n",temp);
  if((temp&0xFF)<=0x17) QSPI.Init.FlashSize = (uint8_t)temp;	//设置 FLASH 大小
	else QSPI.Init.FlashSize = 0x17;
	//hqspi.Init.ClockPrescaler = 1;				//分频    /* ClockPrescaler set to 1, so QSPI clock = 80MHz / (1+1) = 40MHz */
  HAL_QSPI_Init(&QSPI);
 	printf("\r\nExit IS25LPXX_Init\r\n");
}  


/**
  * @brief  Command completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  CmdCplt++;
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  RxCplt++;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
 void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  TxCplt++; 
}

/**
  * @brief  Status Match callbacks
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
  StatusMatch++;
}

