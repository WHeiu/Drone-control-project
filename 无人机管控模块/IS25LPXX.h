#ifndef __IS25LPXX_H
#define __IS25LPXX_H			    
#include "stm32l4xx_hal.h"	   
#include "quadspi.h"

//////Marco definition/////
#define 	TRUE	1
#define 	FALSE	0
//////Device ID////////////
#define IS25LP128	0X9D17 //Manufacturer ID + Device ID, command is 90h


////////////////////////////////////////////////////////////////////////////////// 
//commands
//			Command Name						Code	Command		Instruction Name	Operation	Mode	Byte1	Byte2	Byte3	Byte4	Byte5	Byte6
#define 	IS25LPXX_ReadData3Addr				0x03  //	NORD	Normal Read Mode(3-byte Address)	SPI	A<23:16>	A<15:8>	A<7:0>	Data out		
#define 	IS25LPXX_ReadData4Addr				0x03  //	NORD	Normal Read Mode(4-byte Address)	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_NormalReadData4Addr		0x13  //	4NORD	4-byte Address Normal Read Mode	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_FastRead3Addr				0x0B  //	FRD		Fast Read Mode(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Data out	
#define 	IS25LPXX_FastRead4Addr				0x0B  //	FRD		Fast Read Mode(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Data out
#define 	IS25LPXX_FastReadData4Addr			0x0C  //	4FRD	4-byte Address Fast Read Mode	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Data out
#define 	IS25LPXX_FastReadDIO3Addr			0xBB  //	FRDIO	Fast Read Dual I/O(3-byte Address)	SPI	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1),(2)Dual"	Dual Data out	
#define 	IS25LPXX_FastReadDIO4Addr			0xBB  //	FRDIO	Fast Read Dual I/O(4-byte Address)	SPI	A<31::24>	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1),(2)Dual"	Dual Data out
#define 	IS25LPXX_FastReadDualIO4Addr		0xBC  //	4FRDIO	4-byte Address Fast Read Dual I/O	SPI	A<31::24>	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1),(2)Dual"	Dual Data out
#define 	IS25LPXX_FastReadDO3Addr			0x3B  //	FRDO	Fast Read Dual Output(3-byte Address)	SPI	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out	
#define 	IS25LPXX_FastReadDO4Addr			0x3B  //	FRDO	Fast Read Dual Output(4-byte Address)	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out
#define 	IS25LPXX_FastReadDualO4Addr			0x3C  //	4FRDO	4-byte Address Fast Read Dual Output	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out
#define 	IS25LPXX_FastReadQIO3Addr			0xEB  //	FRQIO	Fast Read Quad I/O(3-byte Address)	SPI QPI	A<23:16>Quad	A<15:8>Quad	A<7:0>Quad	"AXh(1), (2)Quad"	Quad Data out	
#define 	IS25LPXX_FastReadQIO4Addr			0xEB  //	FRQIO	Fast Read Quad I/O(4-byte Address)	SPI QPI	A<31::24>Quad	A<23:16>Quad	A<15:8>Quad	A<7:0>Quad	"AXh(1), (2)Quad"	Quad Data out
#define 	IS25LPXX_FastReadQuadIO4Addr		0xEC  //	4FRQIO	4-byte Address Fast Read Quad I/O	SPI QPI	A<31::24>Quad	A<23:16>Quad	A<15:8>Quad	A<7:0>Quad	"AXh(1), (2)Quad"	Quad Data out
#define 	IS25LPXX_FastReadQO3Addr			0x6B  //	FRQO	Fast Read Quad Output(3-byte Address)	SPI	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Quad Data out	
#define 	IS25LPXX_FastReadQO4Addr			0x6B  //	FRQO	Fast Read Quad Output(4-byte Address)	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Quad Data out
#define 	IS25LPXX_FastReadQuadO4Addr			0x6C  //	4FRQO	4-byte Address Fast Read Quad Output	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Quad Data out
#define 	IS25LPXX_FastReadDTR3Addr			0x0D  //	FRDTR	Fast Read DTR Mode(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out	
#define 	IS25LPXX_FastReadDTR4Addr			0x0D  //	FRDTR	Fast Read DTR Mode(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out
#define 	IS25LPXX_FastReadDTRMode4Addr		0x0E  //	4FRDTR	4-byte Address Fast Read DTR Mode	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Dummy(1) Byte	Dual Data out
#define 	IS25LPXX_FastReadDIODTR3Addr		0xBD  //	FRDDTR	Fast Read Dual I/O DTR(3-byte Address)	SPI	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1), (2)Dual"	Dual Data out	
#define 	IS25LPXX_FastReadDIODTR4Addr		0xBD  //	FRDDTR	Fast Read Dual I/O DTR(4-byte Address)	SPI	A<31::24>	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1), (2)Dual"	Dual Data out
#define 	IS25LPXX_FastReadDualIODTR4Addr		0xBE  //	4FRDDTR	4-byte Address Fast Read Dual I/O DTR	SPI	A<31::24>	A<23:16>Dual	A<15:8>Dual	A<7:0>Dual	"AXh(1), (2)Dual"	Dual Data out
#define 	IS25LPXX_FastReadQIODTR3Addr		0xED  //	FRQDTR	Fast Read Quad I/O DTR(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	"AXh(1), (2)Quad"	Quad Data out	
#define 	IS25LPXX_FastReadQIODTR4Addr		0xED  //	FRQDTR	Fast Read Quad I/O DTR (4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	"AXh(1), (2)Quad"	Quad Data out
#define 	IS25LPXX_FastReadQuadIODTR4Addr		0xEE  //	4FRQDTR	4-byte Address Fast Read Quad I/O DTR	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	"AXh(1), (2)Quad"	Quad Data out
#define 	IS25LPXX_InputPageProgram3Addr		0x02  //	PP		precessed by WREN cmd,Input Page Program(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	PD(256byte)		
#define 	IS25LPXX_InputPageProgram4Addr		0x02  //	PP		precessed by WREN cmd,Input Page Program(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	PD(256byte)	
#define 	IS25LPXX_InputPageProgramWith4Addr	0x12  //	4PP		precessed by WREN cmd,4-byte Address Input Page Program	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	PD(256byte)	
#define 	IS25LPXX_QuadInputPagePro3Addr		0x32  //or 38h	precessed by WREN cmd,//	PPQ	Quad Input Page Program (3-byte Address)	SPI	A<23:16>	A<15:8>	A<7:0>	Quad PD (256byte)		
#define 	IS25LPXX_QuadInputPagePro4Addr		0x32  //or 38h	precessed by WREN cmd,//	PPQ	Quad Input Page Program (4-byte Address)	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Quad PD (256byte)	
#define 	IS25LPXX_QuadInputPageProWith4Addr	0x34  //or 3Eh	precessed by WREN cmd,//	4PPQ	4-byte Address Quad Input Page Program	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Quad PD (256byte)	
#define 	IS25LPXX_SectorErase3Addr			0xD7  //or 20h	precessed by WREN cmd,//	SER	Sector Erase (3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_SectorErase4Addr			0xD7  //or 20h	precessed by WREN cmd,//	SER	Sector Erase (4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_SectroEraseWith4Addr		0x21  //	4SER	precessed by WREN cmd,4-byte Address Sector Erase	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_BlockErase32K3Addr			0x52  //	BER32 (32KB)	precessed by WREN cmd,Block Erase 32Kbyte(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_BlockErase32K4Addr			0x52  //	BER32 (32KB)	precessed by WREN cmd,Block Erase 32Kbyte(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_BlockErase32KWith4Addr		0x5C  //	4BER32 (32KB)	precessed by WREN cmd,4-byte Address Block Erase 32Kbyte	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_BlockErase64K3Addr			0xD8  //	BER64 (64KB)	precessed by WREN cmd,Block Erase 64Kbyte(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_BlockErase64K4Addr			0xD8  //	BER64 (64KB)	precessed by WREN cmd,Block Erase 64Kbyte(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_BlockErase64KWith4Addr		0xDC  //	4BER64 (64KB)	precessed by WREN cmd,4-byte Address Block Erase 64Kbyte	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_ChipErase					0xC7  //or 60h	precessed by WREN cmd,//	CER	Chip Erase	SPI QPI						
#define 	IS25LPXX_WriteEnable				0x06  //	WREN	Write Enable	SPI QPI						
#define 	IS25LPXX_VolSRWriteEnable			0x50  //	VSRWREN	Volatile Status Register Write Enable	SPI QPI						
#define 	IS25LPXX_WriteDisable				0x04  //	WRDI	Write Disable	SPI QPI						
#define 	IS25LPXX_ReadStatusReg				0x05  //	RDSR	Read Status Register	SPI QPI	Data out					
#define 	IS25LPXX_WriteStatusReg				0x01  //	WRSR	precessed by WREN cmd,Write Status Register	SPI QPI	Data in					
#define 	IS25LPXX_ReadFunctionReg			0x48  //	RDFR	Read Function Register	SPI QPI	Data out					
#define 	IS25LPXX_WriteFunctionReg			0x42  //	WRFR	precessed by WREN cmd,Write Function Register	SPI QPI	Data in					
#define 	IS25LPXX_EnterQPIMode				0x35  //	QIOEN	Enter QPI mode	SPI						
#define 	IS25LPXX_ExitQPImode				0xF5  //	QIODI	Exit QPI mode	QPI						
#define 	IS25LPXX_SuspendProOrErase			0x75  //or B0h	//	PERSUS	Suspend during program/erase	SPI QPI						
#define 	IS25LPXX_ResumeProOrErase			0x7A  //or 30h	//	PERRSM	Resume program/erase	SPI QPI						
#define 	IS25LPXX_DeepPowerDown				0xB9  //	DP	Deep Power Down	SPI QPI						
#define 	IS25LPXX_ReadIDReleasePD			0xAB  //	"RDID, RDPD"	Read ID / Release Power Down	SPI QPI	XXh(3)	XXh(3)	XXh(3)	ID7-ID0		
#define 	IS25LPXX_SetReadParasNonVol			0x65  //	SRPNV	precessed by WREN cmd,Set Read Parameters (Non-Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_SetReadParasVol			0xC0  //or 63h	precessed by WREN cmd,//	SRPV	Set Read Parameters (Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_SetExtReadParasNonVol		0x85  //	SERPNV	precessed by WREN cmd,Set Extended Read Parameters (Non-Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_SetExtReadParasVol			0x83  //	SERPV	precessed by WREN cmd,Set Extended Read Parameters (Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_ReadReadParasVol			0x61  //	RDRP	Read Read Parameters (Volatile)	SPI QPI	Data out					
#define 	IS25LPXX_ReadExtReadParasVol		0x81  //	RDERP	Read Extended Read Parameters (Volatile)	SPI QPI	Data out					
#define 	IS25LPXX_ClearExtReadReg			0x82  //	CLERP	Clear Extended Read Register	SPI QPI						
#define 	IS25LPXX_ReadJEDECID				0x9F  //	RDJDID	Read JEDEC ID Command	SPI	MF7-MF0	ID15-ID8	ID7-ID0			
#define 	IS25LPXX_ReadMIDDID					0x90  //	RDMDID	Read Manufacturer & Device ID	SPI QPI	XXh(3)	XXh(3)	00h	MF7-MF0	ID7-ID0	
//#define 	IS25LPXX_ReadDIDMID					0x90  //	RDMDID	Read Manufacturer & Device ID	SPI QPI	XXh(3)	XXh(3)	01h	ID7-ID0	MF7-MF0	
#define 	IS25LPXX_ReadJEDECIDQPImode			0xAF  //	RDJDIDQ	Read JEDEC ID QPI mode	QPI	MF7-MF0	ID15-ID8	ID7-ID0			
#define 	IS25LPXX_ReadUniqueID				0x4B  //	RDUID	Read Unique ID	SPI QPI	A(4)<23:16>	A(4)<15:8>	A(4)<7:0>	Dummy Byte	Data out	
#define 	IS25LPXX_ReadSFDP					0x5A  //	RDSFDP	SFDP Read	SPI	A<23:16>	A<15:8>	A<7:0>	Dummy Byte	Data out	
#define 	IS25LPXX_NOP						0x00  //	NOP	No Operation	SPI QPI						
#define 	IS25LPXX_SoftResetEnable			0x66  //	RSTEN	Software Reset Enable	SPI QPI						
#define 	IS25LPXX_SoftReset					0x99  //	RST	Software Reset	SPI QPI						
#define 	IS25LPXX_EraseInfoRow				0x64  //	IRER	precessed by WREN cmd,Erase Information Row	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_ProgInforRow				0x62  //	precessed by WREN cmd,IRP	Program Information Row	SPI QPI	A<23:16>	A<15:8>	A<7:0>	PD(256byte)		
#define 	IS25LPXX_ReadInfoRow				0x68  //	IRRD	Read Information Row	SPI QPI	A<23:16>	A<15:8>	A<7:0>	Dummy Byte	Data out	
#define 	IS25LPXX_SectorUnlock3Addr			0x26  //	SECUNLOCK	Sector Unlock (3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_SectroUnlock3Addr			0x26  //	SECUNLOCK	Sector Unlock (4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_SectorUnlockWith4Addr		0x25  //	4SECUNLOCK	4-byte Address Sector Unlock	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_SectorLock					0x24  //	SECLOCK	Sector Lock	SPI QPI						
#define 	IS25LPXX_ReadAutoBootReg			0x14  //	RDABR	Read AutoBoot Register	SPI	Data out 1	Data out 2	Data out 3	Data out 4		
#define 	IS25LPXX_WriteAutoBootReg			0x15  //	precessed by WREN cmd,WRABR	Write AutoBoot Register	SPI QPI	Data in 1	Data in 2	Data in 3	Data in 4		
#define 	IS25LPXX_ReadBankAddrRegVol			0x16  //or C8h	//	RDBR	Read Bank Address Register (Volatile)	SPI QPI	Data out					
#define 	IS25LPXX_WriteBankAddrRegVol		0x17  //or C5h	C5h IS precessed by WREN cmd,//	WRBRV	Write Bank Address Register (Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_WriteBankAddrRegNonVol		0x18  //	precessed by WREN cmd,WRBRNV	Write Bank Address Register (Non-Volatile)	SPI QPI	Data in					
#define 	IS25LPXX_Enter4AddrMode				0xB7  //	EN4B	Enter 4-byte Address Mode	SPI QPI						
#define 	IS25LPXX_Exit4AddrMode				0x29  //	EX4B	Exit 4-byte Address Mode	SPI QPI						
#define 	IS25LPXX_ReadDYB3Addr				0xFA  //	RDDYB	Read DYB(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	Data out		
#define 	IS25LPXX_ReadDYB4Addr				0xFA  //	RDDYB	Read DYB(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_ReadDYBWith4Addr			0xE0  //	4RDDYB	4-byte Address Read DYB	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_WriteDYB3Addr				0xFB  //	precessed by WREN cmd,WRDYB	Write DYB(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>	Data in		
#define 	IS25LPXX_WriteDYB4Addr				0xFB  //	precessed by WREN cmd,WRDYB	Write DYB(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data in	
#define 	IS25LPXX_WriteDYBWith4Addr			0xE1  //	precessed by WREN cmd,4WRDYB	4-byte Address Write DYB	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data in	
#define 	IS25LPXX_ReadPPB3Addr				0xFC  //	RDPPB	Read PPB(3-byte Address)	SPI	A<23:16>	A<15:8>	A<7:0>	Data out		
#define 	IS25LPXX_ReadPPB4aDDR				0xFC  //	RDPPB	Read PPB(4-byte Address)	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_ReadPPBWith4Addr			0xE2  //	4RDPPB	4-byte Address Read PPB	SPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>	Data out	
#define 	IS25LPXX_ProgPPB3Addr				0xFD  //	precessed by WREN cmd,PGPPB	Program PPB (Individually)(3-byte Address)	SPI QPI	A<23:16>	A<15:8>	A<7:0>			
#define 	IS25LPXX_ProgPPB4Addr				0xFD  //	PGPPB	Program PPB (Individually)(4-byte Address)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_ProgPPBWith4Addr			0xE3  //	precessed by WREN cmd,4PGPPB	4-byte Address Program PPB (Individually)	SPI QPI	A<31::24>	A<23:16>	A<15:8>	A<7:0>		
#define 	IS25LPXX_RrasePPB					0xE4  //	precessed by WREN cmd,ERPPB	Erase PPB (as a group)	SPI QPI						
#define 	IS25LPXX_ReadASP					0x2B  //	RDASP	Read ASP	SPI QPI	Data out (2 byte)					
#define 	IS25LPXX_ProgASP					0x2F  //	precessed by WREN cmd,PGASP	Program ASP	SPI QPI	PD(2 byte)					
#define 	IS25LPXX_ReadPPBLockBit				0xA7  //	RDPLB	Read PPB Lock Bit	SPI	Data out					
#define 	IS25LPXX_WritePPBLockBit			0xA6  //	precessed by WREN cmd,WRPLB	Write PPB Lock Bit	SPI QPI						
#define 	IS25LPXX_SetFREEZEBit				0x91  //	precessed by WREN cmd,SFRZ	Set FREEZE bit	SPI QPI						
#define 	IS25LPXX_ReadPswd					0xE7  //	RDPWD	Read Password	SPI	Data out (8 byte)					
#define 	IS25LPXX_ProgPswd					0xE8  //	precessed by WREN cmd,PGPWD	Program Password	SPI QPI	PD(8 byte)					
#define 	IS25LPXX_UnLockPswd					0xE9  //	UNPWD	Unlock Password	SPI QPI	Data in (8 byte)					
#define 	IS25LPXX_SetAllDYBBits				0x7E  //	precessed by WREN cmd,GBLK	Set all DYB bits (Gang Sector/ Block Lock)	SPI QPI						
#define 	IS25LPXX_ClearAllDYBBits			0x98  //	precessed by WREN cmd,GBUN	Clear all DYB bits (Gang Sector/ Block Unlock)	SPI QPI						
//					Notes:								
//					1.  The number of dummy cycles depends on the value setting in the Table 6.11 Read Dummy Cycles.								
//					2.  AXh has to be counted as a part of dummy cycles. X means ¡°don¡¯t care¡±.								
//					3.  XX means ¡°don¡¯t care¡±.								
//					4.  A<23:9> are ¡°don¡¯t care¡± and A<8:4> are always ¡°0¡±.								

void IS25LPXX_Init(void);
uint16_t  IS25LPXX_ReadID(uint8_t MDFirst);  	    		//read FLASH device ID
uint8_t IS25LPXX_ReadSR(uint8_t SR) ;        		//Read Status Register
void IS25LPXX_Write_SR(uint8_t *sr,uint8_t NumByteToWrite);  			//Write Status Register
void IS25LPXX_Write_Enable(void);  		//
void IS25LPXX_Write_Disable(void);		//
void IS25LPXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void IS25LPXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   //
void IS25LPXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);//
void IS25LPXX_Erase_Chip(void);    	  	//
void IS25LPXX_Erase_Sector(uint32_t Dst_Addr);	//
void IS25LPXX_Wait_Busy(void);           	//
void IS25LPXX_PowerDown(void);        	//
void IS25LPXX_WAKEUP(void);				//

void IS25LPXX_Quad_Enable(void);		//
void IS25LPXX_Memory_Mapped_Enable(void);	//

#endif

