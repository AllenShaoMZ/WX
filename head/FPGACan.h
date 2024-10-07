#ifndef FPGACAN_H_
#define FPGACAN_H_

#include "mss_sys_services.h"
#include "mss_gpio.h"
#include "typedef.h"

//*********************************************************
//                      �궨��
//*********************************************************
#define		CAN_A_ADDRESS	(0x3019A000)
#define 	CAN_B_ADDRESS	(0x3019B000)

#define 	GM_YK_CMD_ID	(0x000201FF)//���鴦�����ң��ָ��֡ID
#define 	GM_YC_LX_ID		(0x000203FF)//���鴦�����ң����ѯ֡ID
#define		TXJ_YK_CMD_ID	(0x000402FF)//ͨ�Ż�ң��ָ��֡ID
#define		TXJ_YC_LX_ID	(0x000402FF)//ͨ�Ż�ң����ѯ֡ID

#define		GM_YC_LEN		(112)
#define		GM_YC_ID_START	(0x00020400)
#define		GM_YC_ID_END	(0x000204FF)
#define		TXJ_YC_LEN		(16)
#define		TXJ_YC_ID_START	(0x00040300)
#define		TXJ_YC_ID_EDN	(0x000403FF)


//*********************************************************
//                      �����ض���
//********************************************************* 
typedef enum CAN_RECV_TYPE
{
	eGMYC		= 0x11,
	eTXJYC		= 0x22
}E_CAN_RECV_TYPE;

typedef enum CAN_SEND_TYPE
{
	eS_GMYCLX		= 0x11,
	eS_GMYKCMD		= 0x22,
	eS_TXJYCLX		= 0x33,
	eS_TXJYKCMD		= 0x44,
	eS_DEFAULT		= 0xFF
}E_CAN_SEND_TYPE;

typedef struct CAN_FRAME
{
	uint32_t	uiId;
	uint8_t		ucLen;
	uint8_t		aucData[8]; 
	uint8_t		ucRecvFlag;
}T_CAN_FRAME;

typedef struct SINGLE_FRAME_SEND_STATE
{
	Uint32	uiSendFrameCnt;	//����֡����
	Uint32	uiFrameNum;	//��֡��
}T_SINGLE_FRAME_SEND_STATE;

typedef struct CAN_SEND_STATE
{
	Uint8						ucSendFlag;
	E_CAN_SEND_TYPE				eType;
	T_SINGLE_FRAME_SEND_STATE	tGMYCLX;
	T_SINGLE_FRAME_SEND_STATE	tGMYKCMD;
	T_SINGLE_FRAME_SEND_STATE	tTXJYCLX;
	T_SINGLE_FRAME_SEND_STATE	tTXJYKCMD;
}T_CAN_SEND_STATE;

typedef struct GM_YC
{
	uint32_t	uiLastId;
	uint32_t	uiRecvCnt;
	Uint8		ucCheckSum;
	Uint8		ucUpdateFlag;
	Uint8		aucGMYC[112];				//����ң������
}T_GM_YC;

typedef struct TXJ_YC
{
	uint32_t	uiLastId;
	uint32_t	uiRecvCnt;
	Uint8		ucCheckSum;
	Uint8		ucUpdateFlag;
	Uint8		aucTXJYC[16];				//ͨ�Ż�ң������
}T_TXJ_YC;

//*********************************************************
//                      ȫ�ֱ�������
//*********************************************************
extern 	T_CAN_FRAME 		g_tCanRecv[2]; 
extern	T_CAN_SEND_STATE	g_tCanSendState[2];
extern	T_CAN_FRAME 		g_tCanSend_GMYCLX;
extern	T_CAN_FRAME 		g_tCanSend_GMYKCMD;
extern	T_CAN_FRAME			g_tCanSend_TXJYCLX;
extern	T_CAN_FRAME 		g_tCanSend_TXJYKCMD;
extern	T_GM_YC				g_tGMYC[3];
extern	T_TXJ_YC			g_tTXJYC[3];
extern	uint8_t 	SJAcanReceiveCountA; 
extern	uint8_t 	SJAcanReceiveCountB;
//*********************************************************
//                      ��������
//*********************************************************
extern	void 		CANIP_Write(uint32_t uiChannel, uint32_t address, uint8_t ucData);
extern	uint8_t 	CANIP_Read(uint32_t uiChannel, uint32_t address);
extern	void 		CANIP_Init(uint32_t uiChannel);
extern	void 		CANIP_Frame_Send(uint32_t uiChannel, T_CAN_FRAME *ptCanSend);
extern	void 		CANIP_Frame_Recv(uint32_t uiChannel);	
extern	void		CANIP_ISR(uint32_t uiChannel);
extern	void 	SJACanInit_PELI(uint32_t address);
extern	void 	SJACanReceive_PELI(uint32_t address);
extern	void 	SJACanInit_BASIC(uint32_t address);
extern	void 	SJACanSend_BASIC(uint32_t address, uint8_t frameCount, T_CAN_FRAME *ptCanFrame);
extern	void 	SJACanReceive_BASIC(uint32_t address);
extern	void 	SJACanReceive_PELI(uint32_t address);
extern 	void	SJACanSend_PELI(uint32_t address, T_CAN_FRAME *ptCanSend);
extern	void 	CAN_Soft_Init(void);
extern	void 		CAN_Soft_Reset(uint8_t ucCanIdx, E_CAN_RECV_TYPE eCanRecvType);
extern	void 	CAN_Data_Analysis(void);
extern 	void 	CAN_MainData_Send(void);
extern 	void 	CAN_YkData_Send(void);
extern 	void 		CAN_Data_Send(void);

#endif /* FPGACAN_H_ */

