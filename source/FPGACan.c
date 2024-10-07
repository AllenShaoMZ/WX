
#include "FPGACan.h"
#include "extern.h"
#include "includes.h"
#include "os.h"        
#include "mss_gpio.h"  

extern OS_SEM CanSem;  

//*********************************************************
//                      全局变量定义
//*********************************************************
T_CAN_FRAME 		g_tCanRecv[2];
T_CAN_SEND_STATE	g_tCanSendState[2];
T_CAN_FRAME 		g_tCanSend_GMYCLX;			//观瞄遥测轮询帧
T_CAN_FRAME 		g_tCanSend_GMYKCMD;			//观瞄遥控指令帧
T_CAN_FRAME			g_tCanSend_TXJYCLX;			//通信机遥测轮询帧
T_CAN_FRAME 		g_tCanSend_TXJYKCMD;		//通信机遥控指令帧
T_GM_YC				g_tGMYC[3];
T_TXJ_YC			g_tTXJYC[3];
uint8_t 			SJAcanReceiveCountA  = 0;
uint8_t 			SJAcanReceiveCountB  = 0;

/************************************************************************CAN接口层（CANIP）************************************************************************/
/**************************************************************************
   将阻塞延时函数替换为uC/OS-III的OSTimeDly函数
**************************************************************************/

void Delay_c(unsigned int time)
{
    OS_ERR err;
    OSTimeDly(time, OS_OPT_TIME_DLY, &err);  // 延迟 time 个时钟周期
}

/************************************************************************
    函  数  名: CANIP_Write
    函数说明: CANIP 发送数据
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CANIP_Write(uint32_t uiChannel, uint32_t address, uint8_t ucData)
{
	//*((volatile unsigned int *)(0x28100000)) = address;
	
	switch(uiChannel)
	{
		case 0:
			*((volatile unsigned short *)(0x90196000 + (address*16))) = ucData;
			break;
		case 1:
			*((volatile unsigned short *)(0x90197000 + (address*16))) = ucData;
			break;
		default:
			break;
	}
}

/************************************************************************
    函  数  名: CANIP_Read
    函数说明: CANIP 接收数据
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
uint8_t CANIP_Read(uint32_t uiChannel, uint32_t address)
{
	uint8_t ucData = 0;
	//*((volatile unsigned int *)(0x28100000)) = address;
	
	switch(uiChannel)
	{
		case 0:
			ucData = *((volatile unsigned short *)(0x90196000 + (address*16)));
			break;
		case 1:
			ucData = *((volatile unsigned short *)(0x90197000 + (address*16)));
			break;
		default:
			break;
	}
	
	Delay_c(1); // 改为非阻塞的1个时钟周期延迟
	
	return(ucData);
}

/************************************************************************
    函  数  名: CANIP_Init
    函数说明: CANIP 初始化
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CANIP_Init(uint32_t uiChannel)
{
	unsigned int data;
	unsigned int addr;
//	unsigned int IR;

	addr = 0x02;
	data = 0x0A;
	CANIP_Write(uiChannel,addr,data);          //禁止CAN总线
	Delay_c(50);

	addr = 0x03;
	data = 0x01;
	CANIP_Write(uiChannel,addr,data);          //清接收缓存
	Delay_c(1);

	addr = 0x03;
	data = 0x02;
	CANIP_Write(uiChannel,addr,data);          //清发送缓存
	Delay_c(1);

	addr = 0x04;
//	IR = CANIP_Read(uiChannel,addr);           //读中断寄存器状态
	Delay_c(1);

	addr = 0x0E;
	data = 0x0C;
	CANIP_Write(uiChannel,addr,data);          //清发送帧计数
	Delay_c(1);

	addr = 0x0E;
	data = 0x03;
	CANIP_Write(uiChannel,addr,data);          //清接收帧计数
	Delay_c(1);

	addr = 0x02;
	data = 0x05;
	CANIP_Write(uiChannel,addr,data);          //使能CAN总线
	Delay_c(50);

//	data = 0x0C;
//  addr = 0x02;
//  CANIP_Write(uiChannel,addr,data);          //初始化CAN总线
//  Delay_c(50000);

	addr = 0x04;
	data = 0x02 | 0x08;
	CANIP_Write(uiChannel,addr,data);          //使能接收中断，使能发送中断
	Delay_c(5);
}

/************************************************************************
    函  数  名: CANIP_Frame_Send
    函数说明: CANIP 帧发送
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CANIP_Frame_Send(uint32_t uiChannel, T_CAN_FRAME *ptCanSend)	
{
	uint32_t	uiIdx;	
	uint32_t 	uiAddr;
	uint8_t 	ucData;
	uint32_t 	uiId;
	
	/*发送数据前，复位发送缓冲，0x03地址，写入0x2复位发送缓存*/
	uiAddr = 0x03;
	ucData = 0x02;
	CANIP_Write(uiChannel, uiAddr, ucData); 
	
	/*填充帧类型与帧长度*/
	uiAddr = 0x00;
	CANIP_Write(uiChannel, uiAddr, (0x80 | ptCanSend->ucLen));
	
	/*填充Id*/
	uiId = ptCanSend->uiId << 3;
	CANIP_Write(uiChannel, uiAddr, u32_2_u8(uiId, 3));
	CANIP_Write(uiChannel, uiAddr, u32_2_u8(uiId, 2));
	CANIP_Write(uiChannel, uiAddr, u32_2_u8(uiId, 1));
	CANIP_Write(uiChannel, uiAddr, u32_2_u8(uiId, 0));
	
	/*填充数据*/
	for(uiIdx = 0; uiIdx < 8; uiIdx++)
	{
		CANIP_Write(uiChannel, uiAddr, ptCanSend->aucData[uiIdx]);
	}
	
	/*发送数据，0x02地址，写入0x3 启动CAN数据发送*/
	uiAddr = 0x02;
	ucData = 0x03;
	CANIP_Write(uiChannel, uiAddr, ucData);
	
	/*发送数据完成，0x0E地址，写入0xC清发送帧计数*/
	uiAddr = 0x0E;
	ucData = 0x0C;		
	CANIP_Write(uiChannel, uiAddr, ucData);
}

/************************************************************************
    函  数  名: CANIP_Frame_Recv
    函数说明: CANIP 帧接收
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CANIP_Frame_Recv(uint32_t uiChannel)	
{
	uint32_t	uiIdx;
	uint32_t	uiAddr;
	uint32_t	uiId;
	uint8_t		ucDadta;
	uint8_t		aucRecvBuf[13];
	
	/*读取接收缓冲区数据，0x01地址*/
	uiAddr = 0x01;
	for(uiIdx = 0; uiIdx < 13; uiIdx++)
	{
		aucRecvBuf[uiIdx] = CANIP_Read(uiChannel, uiAddr);
	}
	
	/*数据解析*/
	uiId = ( (unsigned int)aucRecvBuf[1] << 21) | ((unsigned int)aucRecvBuf[2] << 13) | ((unsigned int)aucRecvBuf[3] << 5) | ((((unsigned int)aucRecvBuf[4] & 0xF8)) >> 3);		

	g_tCanRecv[uiChannel].uiId			= uiId;
	g_tCanRecv[uiChannel].ucLen			= (aucRecvBuf[0] & 0x0F);
	g_tCanRecv[uiChannel].aucData[0] 	= aucRecvBuf[5];
	g_tCanRecv[uiChannel].aucData[1] 	= aucRecvBuf[6];
	g_tCanRecv[uiChannel].aucData[2] 	= aucRecvBuf[7];
	g_tCanRecv[uiChannel].aucData[3] 	= aucRecvBuf[8];
	g_tCanRecv[uiChannel].aucData[4] 	= aucRecvBuf[9]; 
	g_tCanRecv[uiChannel].aucData[5] 	= aucRecvBuf[10];
	g_tCanRecv[uiChannel].aucData[6] 	= aucRecvBuf[11];
	g_tCanRecv[uiChannel].aucData[7] 	= aucRecvBuf[12];
	
	g_tCanRecv[uiChannel].ucRecvFlag = 1;	
	
	/*读完一帧接收数据帧，0x06地址，写入0x5通知FPGA*/
	uiAddr = 0x06;
	ucDadta = 0x05;
	CANIP_Write(uiChannel, uiAddr, ucDadta);
}

/************************************************************************
    函  数  名: CANIP_ISR
    函数说明: CANIP 中断服务函数
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CANIP_ISR(unsigned int uiChannel)
{
	uint32_t 	uiIR;
	uint32_t	uiAddr;
	
	/*读取中断寄存器状态，0x04地址*/
	uiAddr = 4;
	uiIR = CANIP_Read(uiChannel, uiAddr);	
		
	if((uiIR & 0x02) == 0x02)//接收中断
	{ 
		CANIP_Frame_Recv(uiChannel);
	}
	
	if((uiIR & 0x08) == 0x08)//发送中断
	{
		if(g_tCanSendState[uiChannel].eType == eS_GMYCLX)
		{
			g_tCanSendState[uiChannel].tGMYCLX.uiSendFrameCnt = g_tCanSendState[uiChannel].tGMYCLX.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[uiChannel].eType == eS_GMYKCMD)
		{
			g_tCanSendState[uiChannel].tGMYKCMD.uiSendFrameCnt = g_tCanSendState[uiChannel].tGMYKCMD.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[uiChannel].eType == eS_TXJYCLX)
		{
			g_tCanSendState[uiChannel].tTXJYCLX.uiSendFrameCnt = g_tCanSendState[uiChannel].tTXJYCLX.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[uiChannel].eType == eS_TXJYKCMD)
		{
			g_tCanSendState[uiChannel].tTXJYKCMD.uiSendFrameCnt = g_tCanSendState[uiChannel].tTXJYKCMD.uiSendFrameCnt + 1;
		}
		else
		{
			;
		}
//		switch (g_tCanSendState[uiChannel].eType)
//		{
//			case eS_GMYCLX:
//				g_tCanSendState[uiChannel].tGMYCLX.uiSendFrameCnt++;
//				break;
//			case eS_GMYKCMD:
//				g_tCanSendState[uiChannel].tGMYKCMD.uiSendFrameCnt++;
//				break;
//			case eS_TXJYCLX:
//				g_tCanSendState[uiChannel].tTXJYCLX.uiSendFrameCnt++;
//				break;
//			case eS_TXJYKCMD:
//				g_tCanSendState[uiChannel].tTXJYKCMD.uiSendFrameCnt++;
//				break;
//			default:
//				break;
//		}
		
		g_tCanSendState[uiChannel].eType = eS_DEFAULT;
		g_tCanSendState[uiChannel].ucSendFlag = 1;
	}
}

/************************************************************************CAN接口层（SJA1000）************************************************************************/
void SJACanInit_PELI( uint32_t address) 
{
	uint32_t delay;
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x00;
	*((volatile unsigned short *) address) 				= 0x01;						// 进入复位模式

	for( delay=0; delay<10000; delay++ );											// 等待延时进入复位

	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x1F;
	*((volatile unsigned short *)(address + 0x1F*16)) 	= 0xC8;						// 时钟分频
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x01;
	*((volatile unsigned short *)(address + 0x1*16))  	= 0x0E;						// 命令寄存器
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x04;
	*((volatile unsigned short *)(address + 0x04*16))  	= 0x03;						// 中断使能寄存器
		
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x06;						// 波特率设置  16M - 500K
	*((volatile unsigned short *)(address + 0x06*16))  	= 0x41;						
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x07;
	*((volatile unsigned short *)(address + 0x07*16))  	= 0xBE;
		
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x10;
	*((volatile unsigned short *)(address + 0x10*16)) 	= 0x00;						// ACR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x11;	
	*((volatile unsigned short *)(address + 0x11*16)) 	= 0x00;						// ACR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x12;	
	*((volatile unsigned short *)(address + 0x12*16)) 	= 0x00;						// ACR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x13;	
	*((volatile unsigned short *)(address + 0x13*16)) 	= 0x00;						// ACR寄存器
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x14;
	*((volatile unsigned short *)(address + 0x14*16)) 	= 0xFF;						// AMR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x15;	
	*((volatile unsigned short *)(address + 0x15*16)) 	= 0xFF;						// AMR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x16;	
	*((volatile unsigned short *)(address + 0x16*16)) 	= 0xFF;						// AMR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x17;	
	*((volatile unsigned short *)(address + 0x17*16)) 	= 0xFF;						// AMR寄存器

	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x08;
	*((volatile unsigned short *)(address + 0x08*16))  	= 0xAA;						// 输出控制
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x0F;
	*((volatile unsigned short *)(address + 0x0F*16))  	= 0x00;		
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x0E;
	*((volatile unsigned short *)(address + 0x0E*16))  	= 0x00;		

	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x1D;
	*((volatile unsigned short *)(address + 0x1D*16))  	= 0x00;	

	for( delay=0; delay<10000; delay++ );
	
	*((volatile unsigned short *) CAN_ALE_ADDR)			= 0x00;
	*((volatile unsigned short *) address) 				= 0x00;               		// 进入工作模式
	
	for( delay=0; delay<10000; delay++ );              	
}

/**********************************SJA1000_BASIC初始化*******************************************/
//fpga can init
void SJACanInit_BASIC( uint32_t address) 
{
	uint32_t delay;
//	uint16_t temp; 

	*((volatile unsigned short *) CAN_ALE_ADDR)=0x00;
	*((volatile unsigned short *) address) = 0x01;							// 进入复位模式

	for( delay=0; delay<10000; delay++ );									// 等待延时进入复位

	*((volatile unsigned short *) CAN_ALE_ADDR)=31;
	*((volatile unsigned short *)(address+31*16)) = 0x48;					// 时钟分频
	
	*((volatile unsigned short *) CAN_ALE_ADDR)=1;
	*((volatile unsigned short *)(address+1*16))  = 0x0C;					// 命令寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)=4;
	*((volatile unsigned short *)(address+4*16))  = 0x00;					// ACR寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)=5;
	*((volatile unsigned short *)(address+5*16))  = 0xFF;					// AMR寄存器

	*((volatile unsigned short *) CAN_ALE_ADDR)=6;
	*((volatile unsigned short *)(address+6*16))  = 0x41;//0x40;			// 波特率设置  16M - 500K
	*((volatile unsigned short *) CAN_ALE_ADDR)=7;
	*((volatile unsigned short *)(address+7*16))  = 0xBE;//0x9C;

	*((volatile unsigned short *) CAN_ALE_ADDR)=8;
	*((volatile unsigned short *)(address+8*16))  = 0x1A;					// 输出控制


	*((volatile unsigned short *) CAN_ALE_ADDR)=0;
	*((volatile unsigned short *) address) = 0x42;               			// 进入工作模式
	
	for( delay=0; delay<10000; delay++ );
	
	*((volatile unsigned short *) CAN_ALE_ADDR)=0;
	*((volatile unsigned short *) address) = 0x42;               			// 进入工作模式
	
	for( delay=0; delay<10000; delay++ );

	*((volatile unsigned short *) CAN_ALE_ADDR)=2;
//	temp = *((volatile unsigned short *)(address+2*16));             		// 清状态寄存器
	*((volatile unsigned short *) CAN_ALE_ADDR)=3;
//	temp = *((volatile unsigned short *)(address+3*16));             		// 清中断寄存器          	
}

/***************************************************************************************************/
void GPIO0_IRQHandler(void)
{
//	OS_ERR err;
	uint8_t ir;																									// SJA1000 
	
	MSS_GPIO_clear_irq(MSS_GPIO_0);
	
	*((volatile unsigned short *) CAN_ALE_ADDR)	= 3;
	ir = *((volatile unsigned short *)(CAN_A_ADDRESS+3*16));				
	
	if((ir & 0x01) == 0x01)												
	{
		SJACanReceive_PELI(CAN_A_ADDRESS);									// can receive isr
//		SJACanReceive_BASIC( CAN_A_ADDRESS );
		*((volatile unsigned short *) CAN_ALE_ADDR)=1;
		*((volatile unsigned short *)(CAN_A_ADDRESS+1*16)) = 0x0C;
	}	
	
	if((ir & 0x02) == 0x02)
	{
		if(g_tCanSendState[0].eType == eS_GMYCLX)
		{
			g_tCanSendState[0].tGMYCLX.uiSendFrameCnt = g_tCanSendState[0].tGMYCLX.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[0].eType == eS_GMYKCMD)
		{
			g_tCanSendState[0].tGMYKCMD.uiSendFrameCnt = g_tCanSendState[0].tGMYKCMD.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[0].eType == eS_TXJYCLX)
		{
			g_tCanSendState[0].tTXJYCLX.uiSendFrameCnt = g_tCanSendState[0].tTXJYCLX.uiSendFrameCnt + 1;
		}
		else if(g_tCanSendState[0].eType == eS_TXJYKCMD)
		{
			g_tCanSendState[0].tTXJYKCMD.uiSendFrameCnt = g_tCanSendState[0].tTXJYKCMD.uiSendFrameCnt + 1;
		}
		else
		{
			;
        }
		
		g_tCanSendState[0].eType = eS_DEFAULT;
		g_tCanSendState[0].ucSendFlag = 1;
	}
}

/**********************************SJA1000_BASIC发送*******************************************/

void SJACanSend_BASIC( uint32_t address, uint8_t frameCount, T_CAN_FRAME *ptCanFrame)
{
	uint16_t i;
	uint16_t j;
	uint16_t delay;

	for( j=0; j<frameCount; j++ )																			// 帧数据
	{
		*((volatile unsigned short *) CAN_ALE_ADDR)				= 10;
		*((volatile unsigned short *)(address+10*16)) 			= (ptCanFrame->uiId)>>3;					// ID
		*((volatile unsigned short *) CAN_ALE_ADDR)=11;
		*((volatile unsigned short *)(address+11*16)) 			= ((ptCanFrame->uiId)<<5)|0x08;				// ID

		for( i=0; i<8; i++ )
		{
			*((volatile unsigned short *) CAN_ALE_ADDR)			= (12+i);
			*((volatile unsigned short *)(address+(12+i)*16)) 	= ptCanFrame->aucData[i];
		}

		*((volatile unsigned short *) CAN_ALE_ADDR)				= 1;
		*((volatile unsigned short *)(address+1*16)) 			= 0x01;										// 启动发送

		delay = 0;																																							// 发送超时处理
		*((volatile unsigned short *) CAN_ALE_ADDR)				= 2;
		while( (( *((volatile unsigned short *)(address+2*16)) & 0x0C) != 0x0C) && (delay < 2000 ) )		// 发送成功
		{
			delay++;
			*((volatile unsigned short *) CAN_ALE_ADDR)			= 2;
		}
	}
}

/************************************************************************
    函  数  名: SJACanReceive_BASIC
    函数说明: SJA1000 BASIC模式接收
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void SJACanReceive_BASIC(uint32_t address)
{
	uint16_t i;
	uint32_t id;
	uint8_t  aucRecvBuf[10];
	
	for(i = 0; i < 10; i++)																					
	{
		*((volatile unsigned short *) CAN_ALE_ADDR) = (20+i);
		aucRecvBuf[i] = *((volatile unsigned short *)(address+(20+i)*16));
	}

	id = ( (unsigned long)aucRecvBuf[0]<<3) | ((unsigned long)aucRecvBuf[1]>>5 );		
	
	g_tCanRecv[0].uiId			= id;
	g_tCanRecv[0].aucData[0] 	= aucRecvBuf[2];
	g_tCanRecv[0].aucData[1] 	= aucRecvBuf[3];
	g_tCanRecv[0].aucData[2] 	= aucRecvBuf[4];
	g_tCanRecv[0].aucData[3] 	= aucRecvBuf[5];
	g_tCanRecv[0].aucData[4] 	= aucRecvBuf[6];
	g_tCanRecv[0].aucData[5] 	= aucRecvBuf[7];
	g_tCanRecv[0].aucData[6] 	= aucRecvBuf[8];
	g_tCanRecv[0].aucData[7] 	= aucRecvBuf[9]; 
		
	SJAcanReceiveCountA++;
}


/************************************************************************
    函  数  名: SJACanReceive_PELI
    函数说明: SJA1000 PELI模式接收
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void SJACanReceive_PELI(uint32_t address) 
{
	uint16_t i;
	uint32_t id;
	uint8_t  aucRecvBuf[13];
	
	for(i = 0; i < 13; i++)																					
	{
		*((volatile unsigned short *) CAN_ALE_ADDR)=(16+i);
		aucRecvBuf[i] = *((volatile unsigned short *)(address+(16+i)*16));
	}
	id = ( (unsigned int)aucRecvBuf[1] << 21) | ((unsigned int)aucRecvBuf[2] << 13) | ((unsigned int)aucRecvBuf[3] << 5) | ((((unsigned int)aucRecvBuf[4] & 0xF8)) >> 3);		

	g_tCanRecv[0].uiId			= id;
	g_tCanRecv[0].ucLen			= (aucRecvBuf[0] & 0x0F);
	g_tCanRecv[0].aucData[0] 	= aucRecvBuf[5];
	g_tCanRecv[0].aucData[1] 	= aucRecvBuf[6];
	g_tCanRecv[0].aucData[2] 	= aucRecvBuf[7];
	g_tCanRecv[0].aucData[3] 	= aucRecvBuf[8];
	g_tCanRecv[0].aucData[4] 	= aucRecvBuf[9]; 
	g_tCanRecv[0].aucData[5] 	= aucRecvBuf[10];
	g_tCanRecv[0].aucData[6] 	= aucRecvBuf[11];
	g_tCanRecv[0].aucData[7] 	= aucRecvBuf[12];
	
	g_tCanRecv[0].ucRecvFlag = 1;
	
	SJAcanReceiveCountA++;
}


/************************************************************************
    函  数  名: SJACanSend_PELI
    函数说明: SJA1000 PELI模式发送
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void SJACanSend_PELI(uint32_t address, T_CAN_FRAME *ptCanSend) 
{
	uint16_t	i;
	uint16_t	delay;
	uint32_t	uiId;

	uiId = ptCanSend->uiId << 3;

	*((volatile unsigned short *) CAN_ALE_ADDR)				= 16;
	*((volatile unsigned short *)(address + 16*16)) 		= 0x80 | ptCanSend->ucLen;
	
	/*填充Id*/
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 17;
	*((volatile unsigned short *)(address + 17*16)) 		= u32_2_u8(uiId, 3);					// ID
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 18;
	*((volatile unsigned short *)(address + 18*16)) 		= u32_2_u8(uiId, 2);					// ID
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 19;
	*((volatile unsigned short *)(address + 19*16)) 		= u32_2_u8(uiId, 1);					// ID
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 20;
		*((volatile unsigned short *)(address + 20*16)) 	= u32_2_u8(uiId, 0);					// ID
	
	/*填充数据*/
	for(i = 0; i < 8; i++)
	{
		*((volatile unsigned short *) CAN_ALE_ADDR)			= (21+i);
		*((volatile unsigned short *)(address + (21+i)*16)) = ptCanSend->aucData[i];
	}

	/*命令-发送*/
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 1;
	*((volatile unsigned short *)(address+1*16)) 			= 0x01;									// 启动发送

	/*查询发送状态*/
	delay = 0;																						// 发送超时处理
	*((volatile unsigned short *) CAN_ALE_ADDR)				= 2;
	while( ((*((volatile unsigned short *)(address+2*16)) & 0x0C) != 0x0C) && (delay < 2000) )		// 发送成功
	{
		delay++;
		*((volatile unsigned short *) CAN_ALE_ADDR)			= 2;
	}
}

/************************************************************************CAN应用层************************************************************************/
/************************************************************************
    函  数  名: CAN_Soft_Init
    函数说明: CAN软件初始化
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CAN_Soft_Init(void)
{
	memset((void *)&g_tCanRecv, 			0, sizeof(g_tCanRecv));
	memset((void *)&g_tCanSend_GMYCLX, 		0, sizeof(g_tCanSend_GMYCLX));
	memset((void *)&g_tCanSend_GMYKCMD,		0, sizeof(g_tCanSend_GMYKCMD));
	memset((void *)&g_tCanSend_TXJYKCMD, 	0, sizeof(g_tCanSend_TXJYKCMD));
	memset((void *)&g_tCanSendState, 		0, sizeof(g_tCanSendState));
	
	/*初始化发送状态*/
	g_tCanSendState[0].eType 		= eS_DEFAULT;
	g_tCanSendState[0].ucSendFlag 	= 1;
	g_tCanSendState[1].eType 		= eS_DEFAULT;
	g_tCanSendState[1].ucSendFlag 	= 1;
	
	/*观瞄遥测轮询帧*/
	g_tCanSend_GMYCLX.uiId 			= GM_YC_LX_ID;
	g_tCanSend_GMYCLX.ucLen			= 8;
	g_tCanSend_GMYCLX.aucData[0]	= 0x18;
	g_tCanSend_GMYCLX.aucData[1]	= 0x38;
	g_tCanSend_GMYCLX.aucData[2]	= 0x35;
	g_tCanSend_GMYCLX.aucData[3]	= 0x31;
	g_tCanSend_GMYCLX.aucData[4]	= 0x31;
	g_tCanSend_GMYCLX.aucData[5]	= 0x00;
	g_tCanSend_GMYCLX.aucData[6]	= 0x00;
	g_tCanSend_GMYCLX.aucData[7]	= 0xE7;
	
	/*观瞄遥控指令帧*/
	g_tCanSend_GMYKCMD.uiId			= GM_YK_CMD_ID;
	g_tCanSend_GMYKCMD.ucLen		= 8;
	
	/*通信机遥测轮询帧*/
	g_tCanSend_TXJYCLX.uiId			= TXJ_YC_LX_ID;
	g_tCanSend_TXJYCLX.ucLen		= 8;
	g_tCanSend_TXJYCLX.aucData[0]	= 0x01;
	g_tCanSend_TXJYCLX.aucData[1]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[2]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[3]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[4]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[5]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[6]	= 0x55;
	g_tCanSend_TXJYCLX.aucData[7]	= 0x55;
	
	/*通信机遥控指令帧*/
	g_tCanSend_TXJYKCMD.uiId			= TXJ_YK_CMD_ID;
	g_tCanSend_TXJYKCMD.ucLen		= 8;
	g_tCanSend_TXJYKCMD.aucData[1]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[2]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[3]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[4]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[5]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[6]	= 0x55;
	g_tCanSend_TXJYKCMD.aucData[7]	= 0x55;
}


/************************************************************************
    函  数  名: CAN_Soft_Reset
    函数说明: CAN软件复位
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CAN_Soft_Reset(uint8_t ucCanIdx, E_CAN_RECV_TYPE eCanRecvType)
{
	switch(eCanRecvType)
	{
		case eGMYC:
			g_tGMYC[ucCanIdx].uiLastId 	= 0;
			g_tGMYC[ucCanIdx].uiRecvCnt 	= 0;
			g_tGMYC[ucCanIdx].ucCheckSum 	= 0;
			break;
		case eTXJYC:
			g_tTXJYC[ucCanIdx].uiLastId 	= 0;
			g_tTXJYC[ucCanIdx].uiRecvCnt 	= 0;
			g_tTXJYC[ucCanIdx].ucCheckSum 	= 0;
			break;
		default:
			break;
	}
}

/************************************************************************
    函  数  名: CAN_Data_Analysis
    函数说明: CAN数据解析
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CAN_Data_Analysis(void)
{
    T_CAN_FRAME tCanRecvData;
	uint8_t			ucCanIdx = 0;
	
	for(ucCanIdx = 0; ucCanIdx < 2; ucCanIdx++)
	{
		if(g_tCanRecv[ucCanIdx].ucRecvFlag == 1)
    {
			g_tCanRecv[ucCanIdx].ucRecvFlag = 0;
			
			memcpy(&tCanRecvData, (void *)&g_tCanRecv[ucCanIdx], sizeof(T_CAN_FRAME));
					
			if((tCanRecvData.uiId & 0xFFFFFF00) == 0x00020400)//如果是GM遥测数据
        {
				if(tCanRecvData.uiId == GM_YC_ID_START)
            {
					if((tCanRecvData.ucLen == 8) && (tCanRecvData.aucData[0] == 0x38))
                {
						memcpy(&g_tGMYC[ucCanIdx].aucGMYC[g_tGMYC[ucCanIdx].uiRecvCnt], &tCanRecvData.aucData[0], 8);
						g_tGMYC[ucCanIdx].uiLastId	= tCanRecvData.uiId;
						g_tGMYC[ucCanIdx].uiRecvCnt = g_tGMYC[ucCanIdx].uiRecvCnt + 8;
                }
                else
                {
						CAN_Soft_Reset(ucCanIdx, eGMYC);
                }
            }
				else if(tCanRecvData.uiId == GM_YC_ID_END)
            {
					if(tCanRecvData.ucLen == 8)
                {
						memcpy(&g_tGMYC[ucCanIdx].aucGMYC[g_tGMYC[ucCanIdx].uiRecvCnt], &tCanRecvData.aucData[0], 8);
						g_tGMYC[ucCanIdx].uiLastId		= tCanRecvData.uiId;
						g_tGMYC[ucCanIdx].uiRecvCnt 	= g_tGMYC[ucCanIdx].uiRecvCnt + 8;
						g_tGMYC[ucCanIdx].ucCheckSum 	= AddCheckSum8(&g_tGMYC[ucCanIdx].aucGMYC[1], 110);
						
						if((g_tGMYC[ucCanIdx].ucCheckSum == tCanRecvData.aucData[7]) && (g_tGMYC[ucCanIdx].uiRecvCnt == GM_YC_LEN))
                    {
							g_tGMYC[ucCanIdx].ucUpdateFlag = 1;
							memcpy(&g_tGMYC[2], &g_tGMYC[ucCanIdx], sizeof(g_tGMYC[2]));
							CAN_Soft_Reset(ucCanIdx, eGMYC);
                    }
                    else
                    {
							CAN_Soft_Reset(ucCanIdx, eGMYC);
                    }
                }
            }
            else
            {
					if(tCanRecvData.uiId == (g_tGMYC[ucCanIdx].uiLastId + 1))
                {
						if(tCanRecvData.ucLen == 8)
						{
							memcpy(&g_tGMYC[ucCanIdx].aucGMYC[g_tGMYC[ucCanIdx].uiRecvCnt], &tCanRecvData.aucData[0], 8);
							g_tGMYC[ucCanIdx].uiLastId		= tCanRecvData.uiId;
							g_tGMYC[ucCanIdx].uiRecvCnt 	= g_tGMYC[ucCanIdx].uiRecvCnt + 8;
						}	
						else
                    {
							CAN_Soft_Reset(ucCanIdx, eGMYC);
						}
					}
				}
			}
			else if((tCanRecvData.uiId & 0xFFFFFF00) == 0x00040300)//如果是TXJ遥测数据
			{
				if(tCanRecvData.uiId == TXJ_YC_ID_START)
				{
					if(tCanRecvData.ucLen == 8)
					{
						memcpy(&g_tTXJYC[ucCanIdx].aucTXJYC[g_tTXJYC[ucCanIdx].uiRecvCnt], &tCanRecvData.aucData[0], 8);
						g_tTXJYC[ucCanIdx].uiLastId		= tCanRecvData.uiId;
						g_tTXJYC[ucCanIdx].uiRecvCnt 	= g_tTXJYC[ucCanIdx].uiRecvCnt + 8;
                    }
                    else
                    {
						CAN_Soft_Reset(ucCanIdx, eTXJYC);
                    }
                }
				else if(tCanRecvData.uiId == TXJ_YC_ID_EDN)
				{
					if(tCanRecvData.ucLen == 8)
					{
						memcpy(&g_tTXJYC[ucCanIdx].aucTXJYC[g_tTXJYC[ucCanIdx].uiRecvCnt], &tCanRecvData.aucData[0], 8);
						g_tTXJYC[ucCanIdx].uiLastId		= tCanRecvData.uiId;
						g_tTXJYC[ucCanIdx].uiRecvCnt 	= g_tTXJYC[ucCanIdx].uiRecvCnt + 8;
						
						if(g_tTXJYC[ucCanIdx].uiRecvCnt == TXJ_YC_LEN)
						{
							g_tTXJYC[ucCanIdx].ucUpdateFlag = 1;
							memcpy(&g_tTXJYC[2], &g_tTXJYC[ucCanIdx], sizeof(g_tTXJYC[2]));
							CAN_Soft_Reset(ucCanIdx, eTXJYC);
						}
						else
						{
							CAN_Soft_Reset(ucCanIdx, eTXJYC);
            }
        }
    }
    else
    {
					;
				}
			}
		}
    }
}


/************************************************************************
    函  数  名: CAN_Data_Send
    函数说明: CAN数据发送
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void CAN_Data_Send(void)
{
	uint8_t	ucCanIdx = 0;
	
	for(ucCanIdx = 0; ucCanIdx < 2; ucCanIdx++)
	{
//		if(g_tCanSendState[ucCanIdx].ucSendFlag == 1)
//		{
			/*存在观瞄遥控指令未发送*/
			if(g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum > 0)
			{
				
//				if(g_tCanSendState[ucCanIdx].tGMYKCMD.uiSendFrameCnt < g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum)
//				{
					g_tCanSendState[ucCanIdx].ucSendFlag 	= 0;
					g_tCanSendState[ucCanIdx].eType 		= eS_GMYKCMD;
					CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_GMYKCMD);
					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "GMYKCMD 1 ...\r\n");
				    g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum 	= 0;
					g_tCanSendState[ucCanIdx].tGMYKCMD.uiSendFrameCnt = 0;
//					continue;
//				}
//				else
//				{
//					g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum 	= 0;
//					g_tCanSendState[ucCanIdx].tGMYKCMD.uiSendFrameCnt = 0;
//				}
			}
			
			/*存在通信机遥控指令未发送*/
			if(g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum > 0)
			{
//				if(g_tCanSendState[ucCanIdx].tTXJYKCMD.uiSendFrameCnt < g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum)
//				{
					g_tCanSendState[ucCanIdx].ucSendFlag	= 0;
					g_tCanSendState[ucCanIdx].eType 		= eS_TXJYKCMD;
					CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_TXJYKCMD);
				    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "TXJYKCMD 1 ...\r\n");
				    g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum 		= 0;
					g_tCanSendState[ucCanIdx].tTXJYKCMD.uiSendFrameCnt 	= 0;
//					continue;
//				}
//				else
//				{
//					g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum 		= 0;
//					g_tCanSendState[ucCanIdx].tTXJYKCMD.uiSendFrameCnt 	= 0;
//				}
			}
			
			/*存在观瞄遥测轮询未发送*/
			if(g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum > 0)
			{
//				if(g_tCanSendState[ucCanIdx].tGMYCLX.uiSendFrameCnt < g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum)
//				{
					g_tCanSendState[ucCanIdx].ucSendFlag 	= 0;
					g_tCanSendState[ucCanIdx].eType 		= eS_GMYCLX;
					CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_GMYCLX);
				    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "tGMYCLX 1 ...\r\n");
				    g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum 		= 0;
					g_tCanSendState[ucCanIdx].tGMYCLX.uiSendFrameCnt 	= 0;
//					continue;
//				}
//				else
//				{
//					g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum 		= 0;
//					g_tCanSendState[ucCanIdx].tGMYCLX.uiSendFrameCnt 	= 0;
//				}
			}
			
			/*存在通信机遥测轮询未发送*/
			if(g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum > 0)
			{
//				if(g_tCanSendState[ucCanIdx].tTXJYCLX.uiSendFrameCnt < g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum)
//				{
					g_tCanSendState[ucCanIdx].ucSendFlag 	= 0;
					g_tCanSendState[ucCanIdx].eType 		= eS_TXJYCLX;
					CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_TXJYCLX);
				    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "tTXJYCLX 1 ...\r\n");
				    g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum 	= 0;
					g_tCanSendState[ucCanIdx].tTXJYCLX.uiSendFrameCnt = 0;
//					continue;
//				}
//				else
//				{
//					g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum 	= 0;
//					g_tCanSendState[ucCanIdx].tTXJYCLX.uiSendFrameCnt = 0;
//				}
			}
//		}
	}
}


//void CAN_Data_Send(void)
//{
//	uint8_t	ucCanIdx = 0;
//	
//	for(ucCanIdx = 0; ucCanIdx < 2; ucCanIdx++)
//	{
//		if(g_tCanSendState[ucCanIdx].ucSendFlag == 1)
//		{
//			/* 使用状态机管理 CAN 数据发送 */
//			switch (g_tCanSendState[ucCanIdx].eType)
//			{
//				case eS_GMYKCMD:
//					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "GMYKCMD 1 ...\r\n");
//					if(g_tCanSendState[ucCanIdx].tGMYKCMD.uiSendFrameCnt > g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum)
//					{
//						g_tCanSendState[ucCanIdx].ucSendFlag = 0;
//						CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_GMYKCMD);
//						FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "GMYKCMD 2...\r\n");
//					}
//					else
//					{
//						g_tCanSendState[ucCanIdx].tGMYKCMD.uiFrameNum = 0;
//						g_tCanSendState[ucCanIdx].tGMYKCMD.uiSendFrameCnt = 0;
////						g_tCanSendState[0].eType =eS_DEFAULT;
//					}
//					break;
//				case eS_TXJYKCMD:
//					if(g_tCanSendState[ucCanIdx].tTXJYKCMD.uiSendFrameCnt > g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum)
//					{
//						g_tCanSendState[ucCanIdx].ucSendFlag = 0;
//						CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_TXJYKCMD);
//					}
//					else
//					{
//						g_tCanSendState[ucCanIdx].tTXJYKCMD.uiFrameNum = 0;
//						g_tCanSendState[ucCanIdx].tTXJYKCMD.uiSendFrameCnt = 0;
//					}
//					break;
//				case eS_GMYCLX:
//					if(g_tCanSendState[ucCanIdx].tGMYCLX.uiSendFrameCnt > g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum)
//					{
//						g_tCanSendState[ucCanIdx].ucSendFlag = 0;
//						CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_GMYCLX);
//					}
//					else
//					{
//						g_tCanSendState[ucCanIdx].tGMYCLX.uiFrameNum = 0;
//						g_tCanSendState[ucCanIdx].tGMYCLX.uiSendFrameCnt = 0;
//					}
//					break;
//				case eS_TXJYCLX:
//					if(g_tCanSendState[ucCanIdx].tTXJYCLX.uiSendFrameCnt > g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum)
//					{
//						g_tCanSendState[ucCanIdx].ucSendFlag = 0;
//						CANIP_Frame_Send(ucCanIdx, (T_CAN_FRAME *)&g_tCanSend_TXJYCLX);
//					}
//					else
//					{
//						g_tCanSendState[ucCanIdx].tTXJYCLX.uiFrameNum = 0;
//						g_tCanSendState[ucCanIdx].tTXJYCLX.uiSendFrameCnt = 0;
//					}
//					break;
//				default:
//					break;
//			}
//		}
//	}
//}


