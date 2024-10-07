#ifndef FPGAUART_H_
#define FPGAUART_H_

#include "mss_sys_services.h"
#include "typedef.h"

//*********************************************************
//                      宏定义
//*********************************************************
/*地面遥控帧*/
#define		CX_UART_YK_CMD_PACKHEAD9			((uint16_t)(0x13CD))
#define		CX_UART_YK_CMD_PACKHEAD57			((uint16_t)(0x13EA))
#define		CX_UART_YK_CMD_PACKHEAD249			((uint16_t)(0x13E9))
#define		CX_UART_YK_CMD_PACKSEQ				((uint16_t)(0xC000))
#define		CX_UART_YK_CMD_PACKLEN9				((uint16_t)(0x0009))
#define		CX_UART_YK_CMD_PACKLEN57			((uint16_t)(0x0039))
#define		CX_UART_YK_CMD_PACKLEN249			((uint16_t)(0x00F9))
#define		CX_UART_YK_CMD_PT1_ID				((uint16_t)(0xCACA)) 
#define		CX_UART_YK_CMD_PT2_ID				((uint16_t)(0xC2C2))
	
#define		CX_UART_YK_CMD_PACKHEAD_FIRST		((uint8_t)(0x13))
#define		CX_UART_YK_CMD_RECV_SUCCESS			((uint16_t)0xD2D2)
#define		CX_UART_YK_CMD_RECV_FAIL			((uint16_t)0x2D2D)
#define		CX_UART_YK_CMD_FILL					((uint32_t)0xAAAAAAAA)
	
/*地面注数帧*/	
//.........待完成.........//	
/*有线时间码*/	
#define		CX_UART_TIME_BD_PACKHEAD			((uint16_t)(0x1000))	
#define		CX_UART_TIME_BD_PACKSEQ				((uint16_t)(0xC000))
#define		CX_UART_TIME_BD_PACKLEN				((uint16_t)(0x000B))
#define		CX_UART_TIME_BD_ID					((uint16_t)(0x5555)) 
	
/*有线遥测请求*/	
#define		CX_UART_YC_REQUEST_PACKHEAD			((uint16_t)(0x1001))
#define		CX_UART_YC_REQUEST_PACKSEQ			((uint16_t)(0xC000))
#define		CX_UART_YC_REQUEST_PACKLEN			((uint16_t)(0x0005))
		
/*有线数传请求*/			
#define		CX_UART_SC_REQUEST_PACKHEAD			((uint16_t)(0x1010))
#define		CX_UART_SC_REQUEST_PACKSEQ			((uint16_t)(0xC000))
#define		CX_UART_SC_REQUEST_PACKLEN			((uint16_t)(0x0005))

#define		CX_UART_REQUEST_BD_PACKHEAD_FIRST	((uint8_t)(0x10))

#define		TXJ_UART_SZ_PACKHEAD_FIRST			((uint8_t)(0xEB))
#define		TXJ_UART_SZ_PACKHEAD				((uint16_t)(0xEB90))
#define		TXJ_UART_SZ_DATA_TYPE				((uint8_t)(0x29))
#define		TXJ_UART_SZ_PACKEND					((uint16_t)(0x09D7))
#define		TXJ_UART_YK_CMD_SATESYNWORD			((uint16_t)(0xE14D))
#define		TXJ_UART_YK_CMD_PACKHEAD_FIRST		((uint8_t)(0xE1))
	
//*********************************************************
//                      类型重定义
//********************************************************* 
/*串口数据类型*/		
typedef enum UART_OBJECT
{	
	eTXJ				= 0x01,
	eCX					= 0x02,
	eDC					= 0x03,
	eGZ					= 0x04
}E_UART_OBJECT;

/*串口数据类型*/		
typedef enum UART_DATA_TYPE
{	
	eCX_YKCmd			= 0x11,						//CX遥控指令
	eCX_YCRequest		= 0x22,						//CX遥测请求
	eCX_SCRequest		= 0x33,						//CX数传请求
	eTXJ_YKCmd			= 0x44,						//TXJ遥控指令
	eCX_TimeBD			= 0x55,						//CX时间广播
	eDC_YKCmd			= 0x66,						//DC遥控指令
	eDC_YCRequest		= 0x77,						//DC遥测请求
	eDC_SCRequest		= 0x88						//DC数传请求
}E_UART_DATA_TYPE;


/*地面遥控/注数帧的包顺序类型*/		
typedef enum CX_UART_PACK_TYPE		 
{	
	eMidPack			= 0x00,						//中间包
	eFirstPack			= 0x01,						//首包	
	eEndPack			= 0x02,						//末包 
	eSinglePack 		= 0x03,						//独立包
	eDefault			= 0xFF						//默认独立包 
}E_CX_UART_PACK_TYPE;	

/*通信机数据解析*/
typedef enum {
    STATE_FIND_HEADER,      // 查找包头
    STATE_PARSE_LENGTH,     // 解析数据长度
    STATE_PARSE_TYPE,       // 解析数据类型
    STATE_PARSE_SYNC_WORD,  // 解析卫星同步字
    STATE_PARSE_MODE,       // 解析方式字
    STATE_PARSE_DATA,       // 解析数据
    STATE_PARSE_CRC,        // 解析校验和
    STATE_PARSE_CHECKSUM,   // 解析最终校验和
    STATE_PARSE_END,        // 解析包尾
} UART_STATE;

	
typedef struct CX_UART_STATE
{
	uint8_t				ucYCSeq1;
	uint16_t			usYCSeq2;					//遥测包顺序控制
	uint16_t			usYCSeq;						//遥测包顺序控制
	uint8_t				ucSCSeq1;
	uint16_t			usSCSeq2;					//遥测包顺序控制
	uint16_t			usSCSeq;						//遥测包顺序控制
	uint8_t				ucSCRequestCnt;				//数传请求计数
	uint8_t				ucYCRequestCnt;				//遥测请求计数
	uint8_t				ucRecvCmdCnt;				//接收指令计数
	uint8_t				ucProcessCmdCnt;			//正确处理指令计数
	uint8_t				ucRecvErrorCmdCnt;			//接收错误指令计数
	uint8_t				ucSelfCheckState;			//平台自检状态
	uint16_t			usH;
	uint16_t			usM;
	uint16_t			usL;
	uint16_t			usABCState;
	uint8_t				ucDischargeSwitch;			//放电开关
	uint8_t				uc422State;
	uint8_t				ucSateWorkMode;
}T_CX_UART_STATE;

typedef struct CX_UART_RECV			
{			
    uint16_t    		usPackHeader;				//包头
	uint16_t			usPackSeq;					//包顺序控制
	E_CX_UART_PACK_TYPE	ePackNoType;				//包顺序控制类型
	uint16_t			usPackNo;					//包顺序控制序号
    uint16_t    		usPackLen;					//数据包长度
	uint16_t			usPackID;					//功能识别码
    uint16_t    		usCheckSum;					//校验和
	uint8_t				ucHeaderFindFlag;			//发现包头标志
	uint8_t     		aucRxBuf[256];				//接受数据缓存
	uint32_t    		uiByteNum;					//收到的字节数
}T_CX_UART_RECV;							
						
typedef	struct	CX_UART_YK_CMD					
{					
    uint16_t    		usPackHeader;				//包头
	uint16_t			usPackSeq;					//包顺序控制
	E_CX_UART_PACK_TYPE	ePackNoType;				//包顺序控制类型
	uint16_t			usPackNo;					//包顺序控制序号
	uint16_t    		usPackLen;					//数据包长度
	uint16_t			usPackID;					//功能识别码
	uint16_t    		usCheckSum;					//校验和
	uint16_t			usCmdType;					//指令类型码
	uint8_t				aucCmdData[244];			//遥控应用数据
}T_CX_UART_YK_CMD;	

typedef	struct	CX_UART_TIME_BD					
{					
    uint16_t    		usPackHeader;				//包头
	uint16_t			usPackSeq;					//包顺序控制
	E_CX_UART_PACK_TYPE	ePackNoType;				//包顺序控制类型
	uint16_t			usPackNo;					//包顺序控制序号
	uint16_t    		usPackLen;					//数据包长度
	uint16_t			usPackID;					//功能识别码
	uint16_t    		usCheckSum;					//校验和
	uint8_t				aucTimeData[8];				//时间码数据
}T_CX_UART_TIME_BD;	
			
typedef struct	CX_UART_YK_CMD_STATE			
{		
	uint8_t				aucK521[4];
	uint8_t				aucK522[4];
	uint8_t				aucK523[4];
	uint8_t				aucK524[4];
	uint8_t				aucK525[4];
	uint8_t				aucK526[4];
	uint8_t				aucK527[4];
	uint8_t				aucK528[4];
	uint8_t				aucK529[4];
	uint8_t				aucK5210[4];
	uint8_t				aucK5211[4];
	uint8_t				aucK5212[4];
	uint8_t				aucK5213[4];
	uint8_t				aucK5214[4];
	uint8_t				aucK5215[4];
	uint8_t				aucK5216[4];
	uint8_t				aucK5217[4];
	uint8_t				aucK5218[4];
	uint8_t				aucK5219[4];
	uint8_t				aucK5220[4];
	uint8_t				aucK5221[4];
	uint8_t				aucK5222[4];
	uint8_t				aucK5223[4];
	uint8_t				aucK5224[4];
	uint8_t				aucK5225[4];
	uint8_t				aucK5226[4];
	uint8_t				aucK5227[4];
	uint8_t				aucK5228[4];
	uint8_t				aucK5229[4];
	uint8_t				aucK5230[4];
	uint8_t				aucK5231[4];
	uint8_t				aucK5232[4];
	uint8_t				aucK5233[4];
	uint8_t				aucK5234[4];
	uint8_t				aucK5235[4];
	uint8_t				aucK5236[4];
	uint8_t				aucK5237[4];
	uint8_t				aucK5238[4];
	uint8_t				aucK5239[4];
	uint8_t				aucK5240[4];
	uint8_t				aucK5241[4];
	uint8_t				aucK5242[4];
	uint8_t				aucK5243[4];
	uint8_t				aucK5244[4];
	uint8_t				aucK5245[4];
	uint8_t				aucK5246[4];
	uint8_t				aucK5247[4];
	uint8_t				aucK5248[4];
	uint8_t				aucK5249[4];
	uint8_t				aucK5250[4];
	uint8_t				aucK5251[4];
	uint8_t				aucK5252[4];
	uint8_t				aucK5253[4];
	uint8_t				aucK5254[4];
	uint8_t				aucK5255[4];
	uint8_t				aucK5256[4];
	uint8_t				aucK5257[4];
	uint8_t				aucK5258[4];
	uint8_t				aucK5259[4];
	uint8_t				aucK5260[4];
	uint8_t				aucK5261[4];
	uint8_t				aucK5262[4];
	uint8_t				aucK5263[4];
	uint8_t				aucK5264[4];
	uint8_t				aucK5265[4];
	uint8_t				aucK5266[4];
	uint8_t				aucK5267[4];
	uint8_t				aucK5268[4];
	uint8_t				aucK5269[4];
	uint8_t				aucK5270[4];
	uint8_t				aucK5271[4];
	uint8_t				aucK5272[4];
	uint8_t				aucK5273[4];
	uint8_t				aucK5274[4];
	uint8_t				aucK5275[4];
	uint8_t				aucK5276[4];
	uint8_t				aucK5277[4];
	uint8_t				aucK5278[4];
	uint8_t				aucK5279[4];
	uint8_t				aucK5280[4];
	uint8_t				aucK5281[4];
	uint8_t				aucK5282[4];
	uint8_t				aucK5283[4];
	uint8_t				aucK5284[4];
	uint8_t				aucK5285[4];
	uint8_t				aucK5286[4];
	uint8_t				aucK5287[4];
	uint8_t				aucK5288[4];
	uint8_t				aucK5289[4];
	uint8_t				aucK5290[4];
	uint8_t				aucK5291[4];
	uint8_t				aucK5292[4];
	uint8_t				aucK5293[4];
	uint8_t				aucK5294[4];
	uint8_t				aucK5295[4];
	uint8_t				aucK5296[4];
	uint8_t				aucK5297[4];
	uint8_t				aucK5298[4];
	uint8_t				aucK5299[4];
	uint8_t				aucK52100[4];
	uint8_t				aucK52101[4];
	uint8_t				aucK52102[4];
	uint8_t				aucK52103[4];
	uint8_t				aucK52104[4];
	uint8_t				aucK52105[4];
	uint8_t				aucK52106[4];
	uint8_t				aucK52107[4];
	uint8_t				aucK52108[4];
	uint8_t				aucK52109[4];
	uint8_t				aucK52110[4];
	uint8_t				aucK52111[4];
	uint8_t				aucK52112[4];
	uint8_t				aucK52113[4];
	uint8_t				aucK52114[4];
	uint8_t				aucK52115[4];
	uint8_t				aucK52116[4];
	uint8_t				aucK52117[4];
	uint8_t				aucK52118[4];
	uint8_t				aucK52119[4];
	uint8_t				aucK52120[4];
	uint8_t				aucK52121[4];
	uint8_t				aucK52122[4];
	uint8_t				aucK52123[4];
	uint8_t				aucK52124[4];
	uint8_t				aucK52125[4];
	uint8_t				aucK52126[4];
	uint8_t				aucK52127[4];
	uint8_t				aucK52128[4];
	uint8_t				aucK52129[4];
	uint8_t				aucK52130[4];
	uint8_t				aucK52131[4];
	uint8_t				aucK52132[4];
}T_CX_UART_YK_CMD_STATE;  

typedef struct	CX_UART_STD_TRANSFER_DATA
{
	uint16_t			usGPSWeek;
	uint32_t			uiGPSSecond;
	float				fSateJ2000Rx;
	float				fSateJ2000Ry;
	float				fSateJ2000Rz;
	float				fSateJ2000Vx;
	float				fSateJ2000Vy;
	float				fSateJ2000Vz;
}T_CX_UART_STD_TRANSFER_DATA;

typedef struct	CX_UART_QBXF_COEFF
{
	double				adTimeDur[2];
	int32_t				iOrder;
	int32_t				iSegNum;
	double				dSegDur;
	float				afTimeArray[5];
}T_CX_UART_QBXF_COEFF;

typedef struct	CX_UART_TASK_PARAMs
{
	uint8_t				aucParam[207];
}T_CX_UART_TASK_PARAM;

typedef	struct	CX_UART_YC_REQUEST					
{					
    uint16_t    		usPackHeader;				//包头
	uint16_t			usPackSeq;					//包顺序控制
	E_CX_UART_PACK_TYPE	ePackNoType;				//包顺序控制类型
	uint16_t			usPackNo;					//包顺序控制序号
	uint16_t    		usPackLen;					//数据包长度
	uint8_t				aucBackup[4];				//备用
	uint16_t    		usCheckSum;					//校验和
}T_CX_UART_YC_REQUEST;	

typedef	struct	CX_UART_SC_REQUEST					
{					
    uint16_t    		usPackHeader;				//包头
	uint16_t			usPackSeq;					//包顺序控制
	E_CX_UART_PACK_TYPE	ePackNoType;				//包顺序控制类型
	uint16_t			usPackNo;					//包顺序控制序号
	uint16_t    		usPackLen;					//数据包长度
	uint8_t				aucBackup[4];				//备用
	uint16_t    		usCheckSum;					//校验和
}T_CX_UART_SC_REQUEST;	

typedef struct	CX_UART_YC
{
	uint8_t				aucW0W1_PackHeader[2];
	uint8_t				aucW2W3_PackSeq[2];
	uint8_t				aucW4W5_PackLen[2];
	uint8_t				aucW6W11_HML[6];
	uint8_t				aucW12W15_SateHM[4];
	uint8_t				ucW16_RecvCmdCnt;
	uint8_t				ucW17_SateWorkMode;
	uint8_t				ucW18_Can_XWSoft_State;
	uint8_t				aucW19W21_ErrorCode[3];
	uint8_t				aucW22W23_[2];
	uint8_t				aucW24W25_[2];
	uint8_t				ucW26_Reserved_Timing;
	uint8_t				ucW27_;
	uint8_t				ucW28_;
	uint8_t				ucW29_;
	uint8_t				ucW30_DataState;
	uint8_t				ucW31_Voltage;
	uint8_t				aucW32W33_I[2]; 
	uint8_t				ucW34_GnssVoltage;
	uint8_t				ucW35_XMVoltage;
	uint8_t				ucW36_GZVoltage;
	uint8_t				ucW37_TXJVoltage;
	uint8_t				ucW38_ZHVoltage;
	uint8_t				ucW39_;
	uint8_t				ucW40_;
	uint8_t				ucW41_Signal;	
	uint8_t				ucW42_;
	uint8_t				ucW43_Tempt;
	uint8_t				ucW44_Tempt;  
	uint8_t				ucW45_Tempt;
	uint8_t				ucW46_SelfCheck;
	uint8_t				ucW47_HGP;
	uint8_t				ucW48_ZHState;
	uint8_t				ucW49_ZDBState;
	uint8_t				ucW50_CameraState;
	uint8_t				ucW51_TargetNum;
	uint8_t				ucW52_Light; 
	uint8_t				ucW53_TargetDis;
	uint8_t				ucW54_DMEState;
	uint8_t				aucW55W61_FillParam[7];
	uint8_t				aucW62W63_CheckSum[2];
}T_CX_UART_YC;

typedef struct	CX_UART_SC1
{
	uint8_t				aucW0W1_PackHeader[2];
	uint8_t				aucW2W3_PackId[2];
	uint8_t				aucW4W5_PackSeq[2];
	uint8_t				aucW6W7_PackLen[2];
	uint8_t				aucW8W13_HML[6];
	uint8_t				ucW14_SCRequestCnt;
	uint8_t				ucW15_YCRequestCnt;
	uint8_t				ucW16_RecvCmdCnt;
	uint8_t				ucW17_ProcessCmdCnt;
	uint8_t				ucW18_RecvErrorCmdCnt;
	uint8_t				ucW19_Reserved;
	uint8_t				aucW20W46_[27];
	uint8_t				ucW47_Separationt;
	uint8_t				aucW48W253_[206];
	uint8_t				aucW254W255_CheckSum[2];
}T_CX_UART_SC1;

typedef struct	CX_UART_SC2
{
	uint8_t				aucW0W1_PackHeader[2];
	uint8_t				aucW2W3_PackId[2];
	uint8_t				aucW4W5_PackSeq[2];
	uint8_t				aucW6W7_PackLen[2];
	uint8_t				aucW8W13_HML[6];
	uint8_t				ucW14W253_[240];
	uint8_t				aucW254W255_CheckSum[2];
}T_CX_UART_SC2;

typedef struct TXJ_UART_STATE
{
	uint8_t				ucRecvCmdCnt;				//接收指令计数
	uint8_t				ucProcessCmdCnt;			//正确处理指令计数
	uint8_t				ucRecvErrorCmdCnt;			//接收错误指令计数
}T_TXJ_UART_STATE; 

typedef struct TXJ_UART_RECV			
{	
	/*注入数据帧外围*/
	uint16_t			usPackHead;					//帧头
	uint16_t			usDataLen;					//数据长度
	uint8_t				ucDataType;					//数据类型
	uint16_t			usCheckSum;					//校验和
	uint16_t			usPackEnd;					//帧尾
	/*传输内容*/
    uint16_t    		usSateSynWord;				//卫星同步字
	uint8_t				ucModeWord;					//方式字
	uint8_t				ucPackLen;					//包数据域长度
    uint16_t    		usCRC;						//校验和
	uint8_t				ucHeaderFindFlag;			//发现包头标志
	uint8_t     		aucRxBuf[142];				//接受数据缓存
	uint32_t    		uiByteNum;					//收到的字节数
}T_TXJ_UART_RECV;							
	
typedef	struct	TXJ_UART_YK_CMD					
{					
    uint16_t    		usSateSynWord;				//卫星同步字
	uint8_t				ucModeWord;					//方式字
	uint8_t				ucPackLen;					//包数据域长度
    uint16_t    		usCRC;						//校验和
	uint8_t				ucID;						//1字节设备ID
	uint8_t				aucCmdData[127];			//遥控应用数据
}T_TXJ_UART_YK_CMD;	

typedef struct	TXJ_UART_NAV_CMD
{
	uint16_t			usGPSWeek;
	uint32_t			uiGPSSecond;
	float				fSateJ2000Rx;
	float				fSateJ2000Ry;
	float				fSateJ2000Rz;
	float				fSateJ2000Vx;
	float				fSateJ2000Vy;
	float				fSateJ2000Vz;
}T_TXJ_UART_NAV_CMD;

typedef struct	TXJ_UART_YC
{
	uint8_t				ucPackHeadHigh;
	uint8_t				ucPackHeadLow;
	uint8_t				ucDataLenHigh;
	uint8_t				ucDataLenLow;
	uint8_t				ucDataType;//0x12
	uint8_t				aucW0W1_FrameSyn[2];
	uint8_t				aucW2W7_DominantHead[6];
	uint8_t				ucW8_AuxiliaryHead;
	uint8_t				aucW9W32_AuxiliaryData[24];
	uint8_t				ucW33_PackNo;
	uint8_t				aucW34W125_ValidData[92];
	uint8_t				aucW126W127_CheckSum[2];
	uint8_t				ucCheckSumHigh;
	uint8_t				ucCheckSumLow;
	uint8_t				ucPackEndHigh;
	uint8_t				ucPackEndLow;
}T_TXJ_UART_YC;
                              
//*********************************************************
//                      全局变量申明
//*********************************************************
//CX接收
extern	T_CX_UART_STATE				g_tCXUartState;
extern	T_CX_UART_RECV				g_tCXUartRecv;
extern 	T_CX_UART_YK_CMD			g_tCXUartYKCmd;
extern	T_CX_UART_YC_REQUEST		g_tCXUartYCRequest;
extern	T_CX_UART_SC_REQUEST		g_tCXUartSCRequest;
extern	T_CX_UART_TIME_BD			g_tCXUartTimeBD;
//地测接收
extern	T_CX_UART_RECV				g_tDCUartRecv;
extern	T_CX_UART_YK_CMD			g_tDCUartYKCmd;
extern	T_CX_UART_YC_REQUEST		g_tDCUartYCRequest;
extern	T_CX_UART_SC_REQUEST		g_tDCUartSCRequest;
//共用状态
extern	T_CX_UART_STATE				g_tCXUartState;
extern	T_CX_UART_YK_CMD_STATE		g_tCXUartYKCmdState;
extern	T_CX_UART_STD_TRANSFER_DATA	g_tCXUartStdTransferData;
extern	T_CX_UART_QBXF_COEFF		g_tCXUartQBXFcoeff;
extern	T_CX_UART_TASK_PARAM		g_tCXUartTaskParam;
extern	T_CX_UART_YC_REQUEST		g_tCXUartYCRequest;
extern	T_CX_UART_SC_REQUEST		g_tCXUartSCRequest;
extern	T_CX_UART_YC				g_tCXUartYC;
extern	T_CX_UART_SC1				g_tCXUartSC1;
extern	T_CX_UART_SC2				g_tCXUartSC2;
	
	
extern	T_TXJ_UART_STATE			g_tTXJUartState;
extern	T_TXJ_UART_RECV				g_tTXJUartRecv;
extern	T_TXJ_UART_YK_CMD			g_tTXJUartYKCmd;
extern	T_TXJ_UART_NAV_CMD			g_tTXJUartNavCmd;
extern	T_TXJ_UART_YC				g_tTXJUartYC_ZY[2];
extern	T_TXJ_UART_YC				g_tTXJUartYC_GC[8];

//*********************************************************
//                      函数申明
//*********************************************************
void 		FPGAUart_Init(void);
void 		FPGAUart_SendByte(uint32_t uart_addr,uint8_t data);
void 		FPGAUart_SendString(uint32_t uart_addr,char *str);
void 		FPGAUart_SendArray(uint32_t uart_addr,uint8_t *str,uint16_t len);
void 		delay_1ms(uint16_t cnt);

uint8_t FPGAUart_ReceiveByte(uint32_t uart_rxdata_addr);
void FPGAUart_ReceiveArray(uint32_t uart_rxdata_addr, uint8_t *buffer, uint16_t len);

uint16_t 	Is_FPGA_Uart_RxFifo_Empty(E_UART_OBJECT eUartObject);
void		UART_Soft_Init(void);
void		UART_Reset(E_UART_OBJECT eUartObject);
void 		CX_UART_Data_Analysis(E_UART_OBJECT eUartObject);
void 		DC_UART_Data_Analysis(E_UART_OBJECT eUartObject);
void		UART_Data_Process(E_UART_DATA_TYPE eUartDataType);
void 		CX_UART_YK_Cmd_Process(E_UART_DATA_TYPE eUartDataType);
void 		CX_UART_YC_Request_Process(E_UART_DATA_TYPE eUartDataType);
void 		CX_UART_SC_Request_Process(E_UART_DATA_TYPE eUartDataType);

void 		TXJ_UART_Data_Analysis(E_UART_OBJECT eUartObject);
void 		TXJ_UART_YK_Cmd_Process(void);
void 		TXJ_UART_YC_Pack(void);
void		TXJ_UART_YC_Send(void);

void 		GZ_UART_Data_Analysis(E_UART_OBJECT eUartObject);
#endif /* FPGAUART_H_ */

