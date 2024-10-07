#include "FPGAUart.h"
#include "FPGACan.h"
#include "extern.h"
#include "os.h"  // ����uC/OS-IIIͷ�ļ�

//*********************************************************
//                      ȫ�ֱ�������
//*********************************************************
//CX����
T_CX_UART_RECV				g_tCXUartRecv;
T_CX_UART_YK_CMD			g_tCXUartYKCmd;
T_CX_UART_YC_REQUEST		g_tCXUartYCRequest;
T_CX_UART_SC_REQUEST		g_tCXUartSCRequest;
T_CX_UART_TIME_BD			g_tCXUartTimeBD;
//�ز����
T_CX_UART_RECV				g_tDCUartRecv;
T_CX_UART_YK_CMD			g_tDCUartYKCmd;
T_CX_UART_YC_REQUEST		g_tDCUartYCRequest;
T_CX_UART_SC_REQUEST		g_tDCUartSCRequest;
//����״̬
T_CX_UART_STATE				g_tCXUartState;
T_CX_UART_YK_CMD_STATE		g_tCXUartYKCmdState;
T_CX_UART_STD_TRANSFER_DATA	g_tCXUartStdTransferData;
T_CX_UART_QBXF_COEFF		g_tCXUartQBXFcoeff;
T_CX_UART_TASK_PARAM		g_tCXUartTaskParam;
T_CX_UART_YC				g_tCXUartYC;
T_CX_UART_SC1				g_tCXUartSC1;
T_CX_UART_SC2				g_tCXUartSC2;

//ͨ�Ż����
T_TXJ_UART_STATE			g_tTXJUartState;
T_TXJ_UART_RECV				g_tTXJUartRecv;
T_TXJ_UART_YK_CMD			g_tTXJUartYKCmd;
T_TXJ_UART_NAV_CMD			g_tTXJUartNavCmd;
T_TXJ_UART_YC				g_tTXJUartYC_ZY[2];//������Ҫң�������
T_TXJ_UART_YC				g_tTXJUartYC_GC[8];//���߹���ң�������

/***************************************************************************************************/


//fpga uart init
void FPGAUart_Init(void)
{

    /*ͨ�Ż�115200����У��*/
    *((volatile unsigned short *)UART_TXJ_CTRL_ADDR_DEBUG)     	= 0x88;
    *((volatile unsigned short *)UART_TXJ_CTRL_ADDR_DEBUG)     	= 0x77;
    *((volatile unsigned short *)(UART_TXJ_BOTELV_ADDR_DEBUG))	= 0x015A;
    
	/*����115200����У��*/
    *((volatile unsigned short *)UART_CX_CTRL_ADDR_DEBUG)     	= 0x88;
    *((volatile unsigned short *)UART_CX_CTRL_ADDR_DEBUG)     	= 0x77;
    *((volatile unsigned short *)(UART_CX_BOTELV_ADDR_DEBUG))   = 0x015A;
    
	/*�ز�115200����У��*/
    *((volatile unsigned short *)UART_DC_CTRL_ADDR_DEBUG)     	= 0x88;
    *((volatile unsigned short *)UART_DC_CTRL_ADDR_DEBUG)     	= 0x44;
    *((volatile unsigned short *)(UART_DC_BOTELV_ADDR_DEBUG))   = 0x015A;
	
	/*����115200����У��*/
//    *((volatile unsigned short *)UART_GZ_CTRL_ADDR_DEBUG)     	= 0x88;
//    *((volatile unsigned short *)UART_GZ_CTRL_ADDR_DEBUG)     	= 0x44;
//    *((volatile unsigned short *)(UART_GZ_BOTELV_ADDR_DEBUG))   = 0x015A;
   
}

/***************************************************************************************************/
void FPGAUart_SendByte(uint32_t uart_addr,uint8_t data)
{
    *((volatile unsigned short *)uart_addr)=data;
}


/***************************************************************************************************/
void FPGAUart_SendString(uint32_t uart_addr,char *str)
{
    uint16_t i;

    i=0;
    while( (str[i]!='\0') && (i<1000) )
    {
        *((volatile unsigned short *)uart_addr)=str[i];
        i++;
    }
}
/***************************************************************************************************/
void FPGAUart_SendArray(uint32_t uart_addr,uint8_t *str,uint16_t len)
{
    uint16_t i;

    i=0;
    //while( (str[i]!='\0') && (i<1000) )
    for(i=0;i<len;i++)
    {
        *((volatile unsigned short *)uart_addr)=str[i];
    }
}

/***************************************************************************************************/
void delay_1ms(uint16_t cnt)
{
    OS_ERR err;
    OSTimeDly(cnt, OS_OPT_TIME_DLY, &err);  // ����cnt�������ʱ��uC/OS-III��ʱ�䵥λΪϵͳʱ�ӽ���
}

/*********************************************���պ��� shao******************************************************/
/* ����һ���ֽ� */
uint8_t FPGAUart_ReceiveByte(uint32_t uart_rxdata_addr)
{
    // �ȴ�ֱ�����ռĴ��������ݿɶ�
    while (!(*((volatile unsigned short *)uart_rxdata_addr))); // ����ѯ
    return *((volatile unsigned short *)uart_rxdata_addr); // ��ȡ���յ�������
}

/***************************************************************************************************/
/* ����ָ�����ȵ����� */
void FPGAUart_ReceiveArray(uint32_t uart_rxdata_addr, uint8_t *buffer, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = FPGAUart_ReceiveByte(uart_rxdata_addr); // ���ֽڽ�������
    }
}


/************************************************************************
    ��  ��  ��: Is_FPGA_Uart_RxFifo_Empty
    ����˵��: �ж�UART�Ľ�������fifo�Ƿ�Ϊ��
    �������: ��
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
uint16_t Is_FPGA_Uart_RxFifo_Empty(E_UART_OBJECT eUartObject)
{
  uint16_t	usRxFifoDataNum = 0;
	uint32_t 	uiUartRxStaAddr = 0;
	uint32_t 	uiUartRxNumAddr = 0;
	
	if(eUartObject == eTXJ)
	{
		uiUartRxStaAddr = UART_TXJ_RXSTA_ADDR_DEBUG;
		uiUartRxNumAddr = UART_TXJ_RXNUM_ADDR_DEBUG;
	}
	else if(eUartObject == eCX)
	{
		uiUartRxStaAddr = UART_CX_RXSTA_ADDR_DEBUG;
		uiUartRxNumAddr = UART_CX_RXNUM_ADDR_DEBUG;
	}
	else if(eUartObject == eDC)
	{
		uiUartRxStaAddr = UART_DC_RXSTA_ADDR_DEBUG;
		uiUartRxNumAddr = UART_DC_RXNUM_ADDR_DEBUG;
	}
	else if(eUartObject == eGZ)
	{
		uiUartRxStaAddr = UART_GZ_RXSTA_ADDR_DEBUG;
		uiUartRxNumAddr = UART_GZ_RXNUM_ADDR_DEBUG;
	}
	else
	{
		return usRxFifoDataNum;
	}

    if((FPGA_REG_RW_1BYTE(uiUartRxStaAddr) & 0x01) == 0)
    {
        usRxFifoDataNum = FPGA_REG_RW_2BYTE(uiUartRxNumAddr);//fifo�ǿգ���ȡfifo�е����ݸ���
    }
    else
    {
        usRxFifoDataNum = 0;//fifo��
    }

    return usRxFifoDataNum;
}

/************************************************************************
    ��  ��  ��: UART_Soft_Init
    ����˵��: UART��ر�����ʼ��
    �������: ��
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void UART_Soft_Init(void)
{
	memset((void *)&g_tCXUartRecv, 				0, sizeof(g_tCXUartRecv));
	memset((void *)&g_tCXUartYKCmd, 			0, sizeof(g_tCXUartYKCmd));
	memset((void *)&g_tCXUartYCRequest, 		0, sizeof(g_tCXUartYCRequest));
	memset((void *)&g_tCXUartSCRequest, 		0, sizeof(g_tCXUartSCRequest));	
	memset((void *)&g_tCXUartTimeBD, 			0, sizeof(g_tCXUartTimeBD));	
	
	memset((void *)&g_tDCUartRecv, 				0, sizeof(g_tDCUartRecv));
	memset((void *)&g_tDCUartYKCmd, 			0, sizeof(g_tDCUartYKCmd));
	memset((void *)&g_tDCUartYCRequest, 		0, sizeof(g_tDCUartYCRequest));
	memset((void *)&g_tDCUartSCRequest, 		0, sizeof(g_tDCUartSCRequest));
	
	memset((void *)&g_tCXUartState, 			0, sizeof(g_tCXUartState));
	memset((void *)&g_tCXUartYKCmdState, 		0, sizeof(g_tCXUartYKCmdState));
	memset((void *)&g_tCXUartStdTransferData,	0, sizeof(g_tCXUartStdTransferData));
	memset((void *)&g_tCXUartQBXFcoeff,			0, sizeof(g_tCXUartQBXFcoeff));
	memset((void *)&g_tCXUartTaskParam,			0, sizeof(g_tCXUartTaskParam));
	memset((void *)&g_tCXUartYC, 				0, sizeof(g_tCXUartYC));	
	memset((void *)&g_tCXUartSC1, 				0, sizeof(g_tCXUartSC1));
	memset((void *)&g_tCXUartSC2, 				0, sizeof(g_tCXUartSC2));
	
	g_tCXUartState.ucDischargeSwitch = 0x0A;
	g_tCXUartState.usYCSeq = 0xC000;
	g_tCXUartState.usSCSeq = 0x4000;
		
	memset((void *)&g_tTXJUartState, 			0, sizeof(g_tTXJUartState));
	memset((void *)&g_tTXJUartRecv, 			0, sizeof(g_tTXJUartRecv));
	memset((void *)&g_tTXJUartYKCmd, 			0, sizeof(g_tTXJUartYKCmd));
	memset((void *)&g_tTXJUartNavCmd,			0, sizeof(g_tTXJUartNavCmd));
	memset((void *)&g_tTXJUartYC_ZY,			0, sizeof(g_tTXJUartYC_ZY));
	memset((void *)&g_tTXJUartYC_GC,			0, sizeof(g_tTXJUartYC_GC));
	
}


/************************************************************************
    ��  ��  ��: UART_Reset
    ����˵��: ʵ�ֶ�UART�������ĸ�λ
    �������: ��
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void UART_Reset(E_UART_OBJECT eUartObject)
{
	if(eUartObject == eCX)
	{
		memset((void *)&g_tCXUartRecv.usPackHeader, 0, ((Uint32)(&g_tCXUartRecv.aucRxBuf[0]) - (Uint32)(&g_tCXUartRecv.usPackHeader)));
		memset((void *)&g_tCXUartRecv.aucRxBuf[0], 0, g_tCXUartRecv.uiByteNum);
		g_tCXUartRecv.uiByteNum = 0;
	}
	else if(eUartObject == eDC)
	{
		memset((void *)&g_tDCUartRecv.usPackHeader, 0, ((Uint32)(&g_tDCUartRecv.aucRxBuf[0]) - (Uint32)(&g_tDCUartRecv.usPackHeader)));
		memset((void *)&g_tDCUartRecv.aucRxBuf[0], 0, g_tDCUartRecv.uiByteNum);
		g_tDCUartRecv.uiByteNum = 0;
	}
	else if(eUartObject == eTXJ)
	{
		memset((void *)&g_tTXJUartRecv.usPackHead, 0, ((Uint32)(&g_tTXJUartRecv.aucRxBuf[0]) - (Uint32)(&g_tTXJUartRecv.usPackHead)));
		memset((void *)&g_tTXJUartRecv.aucRxBuf[0], 0, g_tTXJUartRecv.uiByteNum);
		g_tTXJUartRecv.uiByteNum = 0;
	}
	else
	{
		return;
	}
}

/************************************************************************
    ��  ��  ��: UART_Data_Process
    ����˵��: UART���ݴ���
    �������:
    	ucPackageType�����ݰ�����
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void UART_Data_Process(E_UART_DATA_TYPE eUartDataType)
{
	switch (eUartDataType)
	{
		case eCX_YKCmd://ң��ָ��
			CX_UART_YK_Cmd_Process(eCX_YKCmd);
			g_tCXUartState.ucProcessCmdCnt = g_tCXUartState.ucProcessCmdCnt + 1;//��ȷִ��ָ�����+1
			break;
		case eCX_YCRequest://CXң������
			g_tCXUartState.ucYCRequestCnt	= g_tCXUartState.ucYCRequestCnt + 1;//ң���������+1
			CX_UART_YC_Request_Process(eCX_YCRequest);
			break;
		case eCX_SCRequest://CX��������
			g_tCXUartState.ucSCRequestCnt	= g_tCXUartState.ucSCRequestCnt + 1;//�����������+1
			CX_UART_SC_Request_Process(eCX_SCRequest);
			break;
		case eDC_YKCmd://ң��ָ��
			CX_UART_YK_Cmd_Process(eDC_YKCmd);
			g_tCXUartState.ucProcessCmdCnt = g_tCXUartState.ucProcessCmdCnt + 1;//��ȷִ��ָ�����+1
			break;
		case eDC_YCRequest://DCң������
//			g_tCXUartState.ucYCRequestCnt	= g_tCXUartState.ucYCRequestCnt + 1;//ң���������+1	
			CX_UART_YC_Request_Process(eDC_YCRequest);
			break;
		case eDC_SCRequest://DC��������
//			g_tCXUartState.ucSCRequestCnt	= g_tCXUartState.ucSCRequestCnt + 1;//�����������+1
			CX_UART_SC_Request_Process(eDC_SCRequest);
			break;		
		case eTXJ_YKCmd://ͨ�Ż�ң��ָ��
			TXJ_UART_YK_Cmd_Process();
			g_tTXJUartState.ucProcessCmdCnt = g_tTXJUartState.ucProcessCmdCnt + 1;
			break;
		case eCX_TimeBD://ʱ����
			g_tCXUartState.usH 				= u8_2_u16(g_tCXUartTimeBD.aucTimeData[0], g_tCXUartTimeBD.aucTimeData[1]);
			g_tCXUartState.usM 				= u8_2_u16(g_tCXUartTimeBD.aucTimeData[2], g_tCXUartTimeBD.aucTimeData[3]);
			g_tCXUartState.usL 				= u8_2_u16(g_tCXUartTimeBD.aucTimeData[4], g_tCXUartTimeBD.aucTimeData[5]);
			g_tCXUartState.usABCState 		= u8_2_u16(g_tCXUartTimeBD.aucTimeData[6], g_tCXUartTimeBD.aucTimeData[7]);
			break;
		default:
			break;
	}
}


/************************************************************************
    ��  ��  ��: CX_UART_Data_Analysis
    ����˵��: UART�����ֽ����ݣ�У�����
    �������: ��
    �������:
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void CX_UART_Data_Analysis(E_UART_OBJECT eUartObject)
{
	uint32_t			uiUartRxDataAddr	= 0;		//���ڽ�������fifo��ַ
	uint32_t			uiUartTxDataAddr	= 0;		//���ڷ�������fifo��ַ
    uint16_t    		usDataIdx       	= 0;		//���ڽ�����������
    uint16_t    		usRxFifoDataNum 	= 0;		//���ڽ������ݸ���
	uint8_t				ucUartByte 			= 0;		//���ڽ��յ��ĵ��ֽ�����
    uint16_t			usPJPackHead 		= 0;		//ƴ�ӵõ��İ�ͷ
	uint16_t			usPJPackSeq			= 0;		//ƴ�ӵõ��İ�˳�����
	E_CX_UART_PACK_TYPE	eUartPackType		= eDefault;	//ƴ�ӵõ��İ�˳����Ƶ�bit15 downto bit14������
	uint16_t			usPJPackNo			= 0;		//ƴ�ӵõ��İ�˳����Ƶ�bit13 downto bit0�����
    uint16_t			usPJPackLen 		= 0;		//ƴ�ӵõ��İ���
	uint16_t			usPJID				= 0;		//ƴ�ӵõ��Ĺ��ܱ�ʶ��
    uint16_t			usPJCheckSum		= 0;		//ƴ�ӵõ���У���
	uint16_t			usCalCheckSum 		= 0;		//���ݽ������ݼ���õ���У���

	/*��ȡ���ڽ���fifo�е����ݸ���*/
    usRxFifoDataNum = Is_FPGA_Uart_RxFifo_Empty(eUartObject);
	
	switch(eUartObject)
	{
		case eCX://����
			uiUartRxDataAddr = UART_CX_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_CX_TXDATA_ADDR_DEBUG;
			break;
		case eTXJ://ͨ�Ż�
			uiUartRxDataAddr = UART_TXJ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_TXJ_TXDATA_ADDR_DEBUG;
			break;
		case eDC://�ز�
			uiUartRxDataAddr = UART_DC_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_DC_TXDATA_ADDR_DEBUG;
			break;
		case eGZ://����
			uiUartRxDataAddr = UART_GZ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_GZ_TXDATA_ADDR_DEBUG;
			break;
		default:
			break;
	}
    
	/*���UART������������*/
    if(usRxFifoDataNum)
    {
        for(usDataIdx = 0; usDataIdx < usRxFifoDataNum; usDataIdx++)
        {
			/*��UART�����л�ȡһ���ֽ�*/
			ucUartByte = FPGA_REG_RW_1BYTE(uiUartRxDataAddr);
			
			/*���洮�ڽ��յ�������*/
            g_tCXUartRecv.aucRxBuf[g_tCXUartRecv.uiByteNum] = ucUartByte;
            g_tCXUartRecv.uiByteNum 						= g_tCXUartRecv.uiByteNum + 1;
            
			/*�����ͷ�ĸ��ֽڲ��ԣ���ջ��棬����Ѱ�Ұ�ͷ*/
			if(g_tCXUartRecv.uiByteNum == 1)
			{
				if((g_tCXUartRecv.aucRxBuf[0] != CX_UART_YK_CMD_PACKHEAD_FIRST) && (g_tCXUartRecv.aucRxBuf[0] != CX_UART_REQUEST_BD_PACKHEAD_FIRST))
				{
					UART_Reset(eUartObject);
				}
			}
			
			/*������������ֽ�����С��2*/
			if(g_tCXUartRecv.uiByteNum >= 2)
			{
				/*�ж��Ƿ��ҵ���ͷ*/
				if(g_tCXUartRecv.ucHeaderFindFlag == 0)
				{
					/*ƴ�Ӱ�ͷ*/
					usPJPackHead = u8_2_u16(g_tCXUartRecv.aucRxBuf[0], g_tCXUartRecv.aucRxBuf[1]);
					
					/*�жϰ�ͷ�Ƿ���ȷ*/
					switch(usPJPackHead)
					{
						case CX_UART_YK_CMD_PACKHEAD9:						//ң��ָ���ͷ9
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;	
						case CX_UART_YK_CMD_PACKHEAD57:						//ң��ָ���ͷ57
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_YK_CMD_PACKHEAD249:					//ң��ָ���ͷ249
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_YC_REQUEST_PACKHEAD:					//ң�������ͷ
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_SC_REQUEST_PACKHEAD:					//���������ͷ
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_TIME_BD_PACKHEAD:
							g_tCXUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tCXUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						default:
							UART_Reset(eUartObject);//δ���ҵ���ͷ����Ϊ�������󣬰ѻ��渴λ
							break;
					}								
				}
				else//�ҵ���ͷ
				{
					if((g_tCXUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD9) || (g_tCXUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD57) || (g_tCXUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD249))//ң��ָ�������ֻ�����ǵ������ݣ�
					{
						if(g_tCXUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tCXUartRecv.aucRxBuf[2], g_tCXUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YK_CMD_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tCXUartRecv.usPackSeq		= usPJPackSeq;
								g_tCXUartRecv.ePackNoType 	= eUartPackType;
								g_tCXUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tCXUartRecv.uiByteNum > 4)
						{
							if(g_tCXUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tCXUartRecv.aucRxBuf[4], g_tCXUartRecv.aucRxBuf[5]);
								
								if(g_tCXUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD9)
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN9)//16�ֽ�
									{
										g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
								else if(g_tCXUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD57)
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN57)//64�ֽ�
									{
										g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
								else
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN249)//256�ֽ�
									{
										g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
							}
							else if(g_tCXUartRecv.uiByteNum > 6)
							{
								if(g_tCXUartRecv.uiByteNum == 8)
								{
									/*ƴ�ӹ��ܱ�ʶ����*/
									usPJID = u8_2_u16(g_tCXUartRecv.aucRxBuf[6], g_tCXUartRecv.aucRxBuf[7]);
									
									/*ѡ���ܱ�ʶ����*/
									switch(usPJID)
									{
										case CX_UART_YK_CMD_PT1_ID:
											g_tCXUartRecv.usPackID = usPJID;
											break;
										case CX_UART_YK_CMD_PT2_ID:
											g_tCXUartRecv.usPackID = usPJID;
											break;
										default:
											g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
											UART_Reset(eUartObject);//�����ܱ�ʶ�������󣬰ѻ��渴λ
											break;
									}
								}
								else if(g_tCXUartRecv.uiByteNum > 8)
								{
									if(g_tCXUartRecv.uiByteNum == (g_tCXUartRecv.usPackLen + 7))
									{
										/*����ָ�����+1*/
										g_tCXUartState.ucRecvCmdCnt = g_tCXUartState.ucRecvCmdCnt + 1;
										
										/*ƴ��У���*/
										usPJCheckSum = u8_2_u16(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)], g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 2)]);
										
										/*����У���*/
										usCalCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartRecv.aucRxBuf[6], ((g_tCXUartRecv.usPackLen + 1 - 2) / 2));
										
										/*�ж�У���*/
										if(usPJCheckSum == usCalCheckSum)
										{
											if(u8_2_u16(g_tCXUartRecv.aucRxBuf[8], g_tCXUartRecv.aucRxBuf[9]) != 0x9A9A)
											{
												/*У����ȷ�ظ�0xD2D2*/
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_SUCCESS, 1));
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_SUCCESS, 0));
											}
											
											g_tCXUartRecv.usCheckSum = usPJCheckSum;//��¼У���
											
											/*��Ϣ��ֵ������ָ��ṹ��*/
											g_tCXUartYKCmd.usPackHeader	= g_tCXUartRecv.usPackHeader;
											g_tCXUartYKCmd.usPackSeq	= g_tCXUartRecv.usPackSeq;
											g_tCXUartYKCmd.ePackNoType	= g_tCXUartRecv.ePackNoType;												
											g_tCXUartYKCmd.usPackNo		= g_tCXUartRecv.usPackNo;
											g_tCXUartYKCmd.usPackLen	= g_tCXUartRecv.usPackLen;
											g_tCXUartYKCmd.usPackID		= g_tCXUartRecv.usPackID;
											g_tCXUartYKCmd.usCheckSum	= g_tCXUartRecv.usCheckSum;
											g_tCXUartYKCmd.usCmdType	= u8_2_u16(g_tCXUartRecv.aucRxBuf[8], g_tCXUartRecv.aucRxBuf[9]);
											memcpy((void *)&g_tCXUartYKCmd.aucCmdData[0], (void *)&g_tCXUartRecv.aucRxBuf[10], (g_tCXUartRecv.usPackLen + 1 - 6));//����ң��Ӧ������
											
											/*������ɣ���λ����*/
											UART_Reset(eUartObject);
											
											/*�������ݴ���*/
											UART_Data_Process(eCX_YKCmd);
										}
										else
										{
											if(u8_2_u16(g_tCXUartRecv.aucRxBuf[8], g_tCXUartRecv.aucRxBuf[9]) != 0x9A9A)
											{
												/*У�����ظ�0x2D2D*/
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_FAIL, 1));
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_FAIL, 0));	
											}
											
											UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
										}
									}
									else
									{
										;//ֻ�洢����������
									}
								}
								else
								{
									;//�ֽ�7����������
								}
							}
							else
							{
								;//�ֽ�5����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}	
					}
					else if(g_tCXUartRecv.usPackHeader == CX_UART_YC_REQUEST_PACKHEAD)//ң���������
					{
						if(g_tCXUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tCXUartRecv.aucRxBuf[2], g_tCXUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YC_REQUEST_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tCXUartRecv.usPackSeq		= usPJPackSeq;
								g_tCXUartRecv.ePackNoType 	= eUartPackType;
								g_tCXUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tCXUartRecv.uiByteNum > 4)
						{
							if(g_tCXUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tCXUartRecv.aucRxBuf[4], g_tCXUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_YC_REQUEST_PACKLEN)
								{
									g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tCXUartRecv.uiByteNum == (g_tCXUartRecv.usPackLen + 7))
							{								
								/*ƴ��У���*/
								usPJCheckSum = u8_2_u16(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)], g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 2)]);
								
								/*����У���*/
								usCalCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartRecv.aucRxBuf[6], ((g_tCXUartRecv.usPackLen + 1 - 2) / 2));
								
								/*�ж�У���*/
								if(usPJCheckSum == usCalCheckSum)
								{					
									g_tCXUartRecv.usCheckSum = usPJCheckSum;//��¼У���
									
									/*��Ϣ��ֵ������ָ��ṹ��*/
									g_tCXUartYCRequest.usPackHeader	= g_tCXUartRecv.usPackHeader;
									g_tCXUartYCRequest.usPackSeq	= g_tCXUartRecv.usPackSeq;
									g_tCXUartYCRequest.ePackNoType	= g_tCXUartRecv.ePackNoType;												
									g_tCXUartYCRequest.usPackNo		= g_tCXUartRecv.usPackNo;
									g_tCXUartYCRequest.usPackLen	= g_tCXUartRecv.usPackLen;
									g_tCXUartYCRequest.usCheckSum	= g_tCXUartRecv.usCheckSum;
									memcpy((void *)&g_tCXUartYCRequest.aucBackup[0], (void *)&g_tCXUartRecv.aucRxBuf[6], (g_tCXUartRecv.usPackLen + 1 - 2));//����ң�ⱸ������
									
									/*������ɣ���λ����*/
									UART_Reset(eUartObject);
									
									/*�������ݴ���*/
									UART_Data_Process(eCX_YCRequest);
								}
								else
								{
									UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
								}
							}
							else if((g_tCXUartRecv.uiByteNum >= 7) && (g_tCXUartRecv.uiByteNum <= 10)) 
							{
								if(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)] != 0xAA)
								{
									UART_Reset(eUartObject);//�����������󣬰ѻ��渴λ
								}
							}
							else
							{
								;//�ֽ�5���Լ��ֽ�>12����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}
					}
					else if(g_tCXUartRecv.usPackHeader == CX_UART_SC_REQUEST_PACKHEAD)//�����������
					{
						if(g_tCXUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tCXUartRecv.aucRxBuf[2], g_tCXUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_SC_REQUEST_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tCXUartRecv.usPackSeq	= usPJPackSeq;
								g_tCXUartRecv.ePackNoType = eUartPackType;
								g_tCXUartRecv.usPackNo 	= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tCXUartRecv.uiByteNum > 4)
						{
							if(g_tCXUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tCXUartRecv.aucRxBuf[4], g_tCXUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_YC_REQUEST_PACKLEN)
								{
									g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tCXUartRecv.uiByteNum == (g_tCXUartRecv.usPackLen + 7))
							{								
								/*ƴ��У���*/
								usPJCheckSum = u8_2_u16(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)], g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 2)]);
								
								/*����У���*/
								usCalCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartRecv.aucRxBuf[6], ((g_tCXUartRecv.usPackLen + 1 - 2) / 2));
								
								/*�ж�У���*/
								if(usPJCheckSum == usCalCheckSum)
								{
									g_tCXUartRecv.usCheckSum = usPJCheckSum;
									
									/*��Ϣ��ֵ������ָ��ṹ��*/
									g_tCXUartSCRequest.usPackHeader	= g_tCXUartRecv.usPackHeader;
									g_tCXUartSCRequest.usPackSeq	= g_tCXUartRecv.usPackSeq;
									g_tCXUartSCRequest.ePackNoType	= g_tCXUartRecv.ePackNoType;												
									g_tCXUartSCRequest.usPackNo		= g_tCXUartRecv.usPackNo;
									g_tCXUartSCRequest.usPackLen	= g_tCXUartRecv.usPackLen;
									g_tCXUartSCRequest.usCheckSum	= g_tCXUartRecv.usCheckSum;
									memcpy((void *)&g_tCXUartSCRequest.aucBackup[0], (void *)&g_tCXUartRecv.aucRxBuf[6], (g_tCXUartRecv.usPackLen + 1 - 2));//����ң�ⱸ������
									
									/*������ɣ���λ����*/
									UART_Reset(eUartObject);
									
									/*�������ݴ���*/
									UART_Data_Process(eCX_SCRequest);
								}
								else
								{
									UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
								}
							}
							else if((g_tCXUartRecv.uiByteNum >= 7) && (g_tCXUartRecv.uiByteNum <= 10)) 
							{
								if(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)] != 0xAA)
								{
									UART_Reset(eUartObject);//�����������󣬰ѻ��渴λ
								}
							}
							else
							{
								;//�ֽ�5���Լ��ֽ�>12����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}
					}
					else if(g_tCXUartRecv.usPackHeader == CX_UART_TIME_BD_PACKHEAD)//ʱ����
					{
						if(g_tCXUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tCXUartRecv.aucRxBuf[2], g_tCXUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YK_CMD_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tCXUartRecv.usPackSeq		= usPJPackSeq;
								g_tCXUartRecv.ePackNoType 	= eUartPackType;
								g_tCXUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tCXUartRecv.uiByteNum > 4)
						{
							if(g_tCXUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tCXUartRecv.aucRxBuf[4], g_tCXUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_TIME_BD_PACKLEN)//18�ֽ�
								{
									g_tCXUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tCXUartRecv.uiByteNum > 6)
							{
								if(g_tCXUartRecv.uiByteNum == 8)
								{
									/*ƴ�ӹ��ܱ�ʶ����*/
									usPJID = u8_2_u16(g_tCXUartRecv.aucRxBuf[6], g_tCXUartRecv.aucRxBuf[7]);
									
									/*ѡ���ܱ�ʶ����*/
									if(usPJID == CX_UART_TIME_BD_ID)
									{
										g_tCXUartRecv.usPackID = usPJID;
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//�����ܱ�ʶ�������󣬰ѻ��渴λ				
									}
								}
								else if(g_tCXUartRecv.uiByteNum > 8)
								{
									if(g_tCXUartRecv.uiByteNum == (g_tCXUartRecv.usPackLen + 7))
									{										
										/*ƴ��У���*/
										usPJCheckSum = u8_2_u16(g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 1)], g_tCXUartRecv.aucRxBuf[(g_tCXUartRecv.uiByteNum - 2)]);
										
										/*����У���*/
										usCalCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartRecv.aucRxBuf[6], ((g_tCXUartRecv.usPackLen + 1 - 2) / 2));
										
										/*�ж�У���*/
										if(usPJCheckSum == usCalCheckSum)
										{				
											g_tCXUartRecv.usCheckSum = usPJCheckSum;//��¼У���
											
											/*��Ϣ��ֵ������ָ��ṹ��*/
											g_tCXUartTimeBD.usPackHeader	= g_tCXUartRecv.usPackHeader;
											g_tCXUartTimeBD.usPackSeq		= g_tCXUartRecv.usPackSeq;
											g_tCXUartTimeBD.ePackNoType		= g_tCXUartRecv.ePackNoType;												
											g_tCXUartTimeBD.usPackNo		= g_tCXUartRecv.usPackNo;
											g_tCXUartTimeBD.usPackLen		= g_tCXUartRecv.usPackLen;
											g_tCXUartTimeBD.usPackID		= g_tCXUartRecv.usPackID; 
											g_tCXUartTimeBD.usCheckSum		= g_tCXUartRecv.usCheckSum; 

											memcpy((void *)&g_tCXUartTimeBD.aucTimeData[0], (void *)&g_tCXUartRecv.aucRxBuf[8], (g_tCXUartRecv.usPackLen + 1 - 4));//����ң��Ӧ������
											
											/*������ɣ���λ����*/
											UART_Reset(eUartObject);
											
											/*�������ݴ���*/
											UART_Data_Process(eCX_TimeBD);
										}
										else
										{				
											UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
										}
									}
									else
									{
										;//ֻ�洢����������
									}
								}
								else
								{
									;//�ֽ�7����������
								}
							}
							else
							{
								;//�ֽ�5����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}	
					}
					else
					{
						;//��������
					}
				}
			}
		}
    }
}

/************************************************************************
    ��  ��  ��: UART_DC_UART_Data_Analysis
    ����˵��: UART�����ֽ����ݣ�У�����
    �������: ��
    �������:
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void DC_UART_Data_Analysis(E_UART_OBJECT eUartObject)
{
	uint32_t			uiUartRxDataAddr	= 0;      	//���ڽ�������fifo��ַ
	uint32_t			uiUartTxDataAddr	= 0;       	//���ڷ�������fifo��ַ
    uint16_t    		usDataIdx       	= 0;       	//���ڽ�����������
    uint16_t    		usRxFifoDataNum 	= 0;       	//���ڽ������ݸ���
	uint8_t				ucUartByte 			= 0;		//���ڽ��յ��ĵ��ֽ�����
    uint16_t			usPJPackHead 		= 0;		//ƴ�ӵõ��İ�ͷ
	uint16_t			usPJPackSeq			= 0;		//ƴ�ӵõ��İ�˳�����
	E_CX_UART_PACK_TYPE	eUartPackType		= eDefault;	//ƴ�ӵõ��İ�˳����Ƶ�bit15 downto bit14������
	uint16_t			usPJPackNo			= 0;		//ƴ�ӵõ��İ�˳����Ƶ�bit13 downto bit0�����
    uint16_t			usPJPackLen 		= 0;		//ƴ�ӵõ��İ���
	uint16_t			usPJID				= 0;		//ƴ�ӵõ��Ĺ��ܱ�ʶ��
    uint16_t			usPJCheckSum		= 0;		//ƴ�ӵõ���У���
	uint16_t			usCalCheckSum 		= 0;		//���ݽ������ݼ���õ���У���

	/*��ȡ���ڽ���fifo�е����ݸ���*/
    usRxFifoDataNum = Is_FPGA_Uart_RxFifo_Empty(eUartObject);
	
	switch(eUartObject)
	{
		case eCX://����
			uiUartRxDataAddr = UART_CX_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_CX_TXDATA_ADDR_DEBUG;
			break;
		case eTXJ://ͨ�Ż�
			uiUartRxDataAddr = UART_TXJ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_TXJ_TXDATA_ADDR_DEBUG;
			break;
		case eDC://�ز�
			uiUartRxDataAddr = UART_DC_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_DC_TXDATA_ADDR_DEBUG;
			break;
		case eGZ://����
			uiUartRxDataAddr = UART_GZ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_GZ_TXDATA_ADDR_DEBUG;
			break;
		default:
			break;
	}
    
	/*���UART������������*/
    if(usRxFifoDataNum)
    {
        for(usDataIdx = 0; usDataIdx < usRxFifoDataNum; usDataIdx++)
        {
			/*��UART�����л�ȡһ���ֽ�*/
			ucUartByte = FPGA_REG_RW_1BYTE(uiUartRxDataAddr);
			
			/*���洮�ڽ��յ�������*/
            g_tDCUartRecv.aucRxBuf[g_tDCUartRecv.uiByteNum] = ucUartByte;
            g_tDCUartRecv.uiByteNum 						= g_tDCUartRecv.uiByteNum + 1;
            
			/*�����ͷ�ĸ��ֽڲ��ԣ���ջ��棬����Ѱ�Ұ�ͷ*/
			if(g_tDCUartRecv.uiByteNum == 1)
			{
				if((g_tDCUartRecv.aucRxBuf[0] != CX_UART_YK_CMD_PACKHEAD_FIRST) && (g_tDCUartRecv.aucRxBuf[0] != CX_UART_REQUEST_BD_PACKHEAD_FIRST))
				{
					UART_Reset(eUartObject);
				}
			}
			
			/*������������ֽ�����С��2*/
			if(g_tDCUartRecv.uiByteNum >= 2)
			{
				/*�ж��Ƿ��ҵ���ͷ*/
				if(g_tDCUartRecv.ucHeaderFindFlag == 0)
				{
					/*ƴ�Ӱ�ͷ*/
					usPJPackHead = u8_2_u16(g_tDCUartRecv.aucRxBuf[0], g_tDCUartRecv.aucRxBuf[1]);
					
					/*�жϰ�ͷ�Ƿ���ȷ*/
					switch(usPJPackHead)
					{
						case CX_UART_YK_CMD_PACKHEAD9:						//ң��ָ���ͷ9
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;	
						case CX_UART_YK_CMD_PACKHEAD57:						//ң��ָ���ͷ57
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_YK_CMD_PACKHEAD249:					//ң��ָ���ͷ249
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_YC_REQUEST_PACKHEAD:					//ң�������ͷ
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_SC_REQUEST_PACKHEAD:					//���������ͷ
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						case CX_UART_TIME_BD_PACKHEAD:
							g_tDCUartRecv.usPackHeader   	= usPJPackHead;	//��¼��ͷ
							g_tDCUartRecv.ucHeaderFindFlag	= 0xFF;			//��ʾ�ҵ���ͷ
							break;
						default:
							UART_Reset(eUartObject);//δ���ҵ���ͷ����Ϊ�������󣬰ѻ��渴λ
							break;
					}								
				}
				else//�ҵ���ͷ
				{
					if((g_tDCUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD9) || (g_tDCUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD57) || (g_tDCUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD249))//ң��ָ�������ֻ�����ǵ������ݣ�
					{
						if(g_tDCUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tDCUartRecv.aucRxBuf[2], g_tDCUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YK_CMD_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tDCUartRecv.usPackSeq		= usPJPackSeq;
								g_tDCUartRecv.ePackNoType 	= eUartPackType;
								g_tDCUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tDCUartRecv.uiByteNum > 4)
						{
							if(g_tDCUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tDCUartRecv.aucRxBuf[4], g_tDCUartRecv.aucRxBuf[5]);
								
								if(g_tDCUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD9)
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN9)//16�ֽ�
									{
										g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
								else if(g_tDCUartRecv.usPackHeader == CX_UART_YK_CMD_PACKHEAD57)
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN57)//64�ֽ�
									{
										g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
								else
								{
									if(usPJPackLen == CX_UART_YK_CMD_PACKLEN249)//256�ֽ�
									{
										g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
									}
								}
							}
							else if(g_tDCUartRecv.uiByteNum > 6)
							{
								if(g_tDCUartRecv.uiByteNum == 8)
								{
									/*ƴ�ӹ��ܱ�ʶ����*/
									usPJID = u8_2_u16(g_tDCUartRecv.aucRxBuf[6], g_tDCUartRecv.aucRxBuf[7]);
									
									/*ѡ���ܱ�ʶ����*/
									switch(usPJID)
									{
										case CX_UART_YK_CMD_PT1_ID:
											g_tDCUartRecv.usPackID = usPJID;
											break;
										case CX_UART_YK_CMD_PT2_ID:
											g_tDCUartRecv.usPackID = usPJID;
											break;
										default:
											g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
											UART_Reset(eUartObject);//�����ܱ�ʶ�������󣬰ѻ��渴λ
											break;
									}
								}
								else if(g_tDCUartRecv.uiByteNum > 8)
								{
									if(g_tDCUartRecv.uiByteNum == (g_tDCUartRecv.usPackLen + 7))
									{
										/*����ָ�����+1*/
										g_tCXUartState.ucRecvCmdCnt = g_tCXUartState.ucRecvCmdCnt + 1;
										
										/*ƴ��У���*/
										usPJCheckSum = u8_2_u16(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 2)], g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)]);
										
										/*����У���*/
										usCalCheckSum = XorCheckSum16((uint16_t *)&g_tDCUartRecv.aucRxBuf[6], ((g_tDCUartRecv.usPackLen + 1 - 2) / 2));
										
										/*�ж�У���*/
										if(usPJCheckSum == usCalCheckSum)
										{
											if(u8_2_u16(g_tDCUartRecv.aucRxBuf[8], g_tDCUartRecv.aucRxBuf[9]) != 0x9A9A)
											{
												/*У����ȷ�ظ�0xD2D2*/
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_SUCCESS, 1));
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_SUCCESS, 0));
											}
											
											g_tDCUartRecv.usCheckSum = usPJCheckSum;//��¼У���
											
											/*��Ϣ��ֵ������ָ��ṹ��*/
											g_tDCUartYKCmd.usPackHeader	= g_tDCUartRecv.usPackHeader;
											g_tDCUartYKCmd.usPackSeq	= g_tDCUartRecv.usPackSeq;
											g_tDCUartYKCmd.ePackNoType	= g_tDCUartRecv.ePackNoType;												
											g_tDCUartYKCmd.usPackNo		= g_tDCUartRecv.usPackNo;
											g_tDCUartYKCmd.usPackLen	= g_tDCUartRecv.usPackLen;
											g_tDCUartYKCmd.usPackID		= g_tDCUartRecv.usPackID;
											g_tDCUartYKCmd.usCheckSum	= g_tDCUartRecv.usCheckSum;
											g_tDCUartYKCmd.usCmdType	= u8_2_u16(g_tDCUartRecv.aucRxBuf[8], g_tDCUartRecv.aucRxBuf[9]);
											memcpy((void *)&g_tDCUartYKCmd.aucCmdData[0], (void *)&g_tDCUartRecv.aucRxBuf[10], (g_tDCUartRecv.usPackLen + 1 - 6));//����ң��Ӧ������
											
											/*������ɣ���λ����*/
											UART_Reset(eUartObject);
											
											/*�������ݴ���*/
											UART_Data_Process(eDC_YKCmd);
										}
										else
										{
											if(u8_2_u16(g_tDCUartRecv.aucRxBuf[8], g_tDCUartRecv.aucRxBuf[9]) != 0x9A9A)
											{
												/*У�����ظ�0x2D2D*/
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_FAIL, 1));
												FPGAUart_SendByte(uiUartTxDataAddr, u16_2_u8(CX_UART_YK_CMD_RECV_FAIL, 0));	
											}
											
											UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
										}
									}
									else
									{
										;//ֻ�洢����������
									}
								}
								else
								{
									;//�ֽ�7����������
								}
							}
							else
							{
								;//�ֽ�5����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}	
					}
					else if(g_tDCUartRecv.usPackHeader == CX_UART_YC_REQUEST_PACKHEAD)//ң���������
					{
						if(g_tDCUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tDCUartRecv.aucRxBuf[2], g_tDCUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YC_REQUEST_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tDCUartRecv.usPackSeq		= usPJPackSeq;
								g_tDCUartRecv.ePackNoType 	= eUartPackType;
								g_tDCUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tDCUartRecv.uiByteNum > 4)
						{
							if(g_tDCUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tDCUartRecv.aucRxBuf[4], g_tDCUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_YC_REQUEST_PACKLEN)
								{
									g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tDCUartRecv.uiByteNum == (g_tDCUartRecv.usPackLen + 7))
							{							
								/*ƴ��У���*/
								usPJCheckSum = u8_2_u16(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 2)], g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)]);
								
								/*����У���*/
								usCalCheckSum = XorCheckSum16((uint16_t *)&g_tDCUartRecv.aucRxBuf[6], ((g_tDCUartRecv.usPackLen + 1 - 2) / 2));
								
								/*�ж�У���*/
								if(usPJCheckSum == usCalCheckSum)
								{					
									g_tDCUartRecv.usCheckSum = usPJCheckSum;//��¼У���
									
									/*��Ϣ��ֵ������ָ��ṹ��*/
									g_tDCUartYCRequest.usPackHeader	= g_tDCUartRecv.usPackHeader;
									g_tDCUartYCRequest.usPackSeq	= g_tDCUartRecv.usPackSeq;
									g_tDCUartYCRequest.ePackNoType	= g_tDCUartRecv.ePackNoType;												
									g_tDCUartYCRequest.usPackNo		= g_tDCUartRecv.usPackNo;
									g_tDCUartYCRequest.usPackLen	= g_tDCUartRecv.usPackLen;
									g_tDCUartYCRequest.usCheckSum	= g_tDCUartRecv.usCheckSum;
									memcpy((void *)&g_tDCUartYCRequest.aucBackup[0], (void *)&g_tDCUartRecv.aucRxBuf[6], (g_tDCUartRecv.usPackLen + 1 - 2));//����ң�ⱸ������
									
									/*������ɣ���λ����*/
									UART_Reset(eUartObject);
									
									/*�������ݴ���*/
									UART_Data_Process(eDC_YCRequest);
								}
								else
								{
									UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
								}
							}
							else if((g_tDCUartRecv.uiByteNum >= 7) && (g_tDCUartRecv.uiByteNum <= 10)) 
							{
								if(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)] != 0xAA)
								{
									UART_Reset(eUartObject);//�����������󣬰ѻ��渴λ
								}
							}
							else
							{
								;//�ֽ�5���Լ��ֽ�>12����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}
					}
					else if(g_tDCUartRecv.usPackHeader == CX_UART_SC_REQUEST_PACKHEAD)//�����������
					{
						if(g_tDCUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tDCUartRecv.aucRxBuf[2], g_tDCUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_SC_REQUEST_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tDCUartRecv.usPackSeq	= usPJPackSeq;
								g_tDCUartRecv.ePackNoType = eUartPackType;
								g_tDCUartRecv.usPackNo 	= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tDCUartRecv.uiByteNum > 4)
						{
							if(g_tDCUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tDCUartRecv.aucRxBuf[4], g_tDCUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_YC_REQUEST_PACKLEN)
								{
									g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tDCUartRecv.uiByteNum == (g_tDCUartRecv.usPackLen + 7))
							{								
								/*ƴ��У���*/
								usPJCheckSum = u8_2_u16(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 2)], g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)]);
								
								/*����У���*/
								usCalCheckSum = XorCheckSum16((uint16_t *)&g_tDCUartRecv.aucRxBuf[6], ((g_tDCUartRecv.usPackLen + 1 - 2) / 2));
								
								/*�ж�У���*/
								if(usPJCheckSum == usCalCheckSum)
								{
									g_tDCUartRecv.usCheckSum = usPJCheckSum;
									
									/*��Ϣ��ֵ������ָ��ṹ��*/
									g_tDCUartSCRequest.usPackHeader	= g_tDCUartRecv.usPackHeader;
									g_tDCUartSCRequest.usPackSeq	= g_tDCUartRecv.usPackSeq;
									g_tDCUartSCRequest.ePackNoType	= g_tDCUartRecv.ePackNoType;												
									g_tDCUartSCRequest.usPackNo		= g_tDCUartRecv.usPackNo;
									g_tDCUartSCRequest.usPackLen	= g_tDCUartRecv.usPackLen;
									g_tDCUartSCRequest.usCheckSum	= g_tDCUartRecv.usCheckSum;
									memcpy((void *)&g_tDCUartSCRequest.aucBackup[0], (void *)&g_tDCUartRecv.aucRxBuf[6], (g_tDCUartRecv.usPackLen + 1 - 2));//����ң�ⱸ������
									
									/*������ɣ���λ����*/
									UART_Reset(eUartObject);
									
									/*�������ݴ���*/
									UART_Data_Process(eDC_SCRequest);
								}
								else
								{
									UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
								}
							}
							else if((g_tDCUartRecv.uiByteNum >= 7) && (g_tDCUartRecv.uiByteNum <= 10)) 
							{
								if(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)] != 0xAA)
								{
									UART_Reset(eUartObject);//�����������󣬰ѻ��渴λ
								}
							}
							else
							{
								;//�ֽ�5���Լ��ֽ�>12����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}
					}
					else if(g_tDCUartRecv.usPackHeader == CX_UART_TIME_BD_PACKHEAD)//ʱ����
					{
						if(g_tDCUartRecv.uiByteNum == 4)
						{
							/*ƴ�Ӱ�˳��������*/
							usPJPackSeq 	= u8_2_u16(g_tDCUartRecv.aucRxBuf[2], g_tDCUartRecv.aucRxBuf[3]);//��˳�����
							eUartPackType 	= (E_CX_UART_PACK_TYPE)CSL_FEXTR(usPJPackSeq, 15, 14);//������
							usPJPackNo		= (uint16_t)CSL_FEXTR(usPJPackSeq, 13, 0);//������
							
							/*�жϰ����*/
							if(usPJPackSeq == CX_UART_YK_CMD_PACKSEQ)//�Ƿ��Ƕ�����
							{
								g_tDCUartRecv.usPackSeq		= usPJPackSeq;
								g_tDCUartRecv.ePackNoType 	= eUartPackType;
								g_tDCUartRecv.usPackNo 		= usPJPackNo;
							}
							else
							{
//								UART_Reset(eUartObject);//��˳��������󣬰ѻ��渴λ
							}
						}
						else if(g_tDCUartRecv.uiByteNum > 4)
						{
							if(g_tDCUartRecv.uiByteNum == 6)
							{
								/*ƴ�Ӱ���*/
								usPJPackLen = u8_2_u16(g_tDCUartRecv.aucRxBuf[4], g_tDCUartRecv.aucRxBuf[5]);
								
								if(usPJPackLen == CX_UART_TIME_BD_PACKLEN)//18�ֽ�
								{
									g_tDCUartRecv.usPackLen = usPJPackLen;//��¼����
								}
								else
								{
									g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
									UART_Reset(eUartObject);//���������󣬰ѻ��渴λ
								}
							}
							else if(g_tDCUartRecv.uiByteNum > 6)
							{
								if(g_tDCUartRecv.uiByteNum == 8)
								{
									/*ƴ�ӹ��ܱ�ʶ����*/
									usPJID = u8_2_u16(g_tDCUartRecv.aucRxBuf[6], g_tDCUartRecv.aucRxBuf[7]);
									
									/*ѡ���ܱ�ʶ����*/
									if(usPJID == CX_UART_TIME_BD_ID)
									{
										g_tDCUartRecv.usPackID = usPJID;
									}
									else
									{
										g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1
										UART_Reset(eUartObject);//�����ܱ�ʶ�������󣬰ѻ��渴λ				
									}
								}
								else if(g_tDCUartRecv.uiByteNum > 8)
								{
									if(g_tDCUartRecv.uiByteNum == (g_tDCUartRecv.usPackLen + 7))
									{
										/*����ָ�����+1*/
//										g_tCXUartState.ucRecvCmdCnt = g_tCXUartState.ucRecvCmdCnt + 1;
										
										/*ƴ��У���*/
										usPJCheckSum = u8_2_u16(g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 2)], g_tDCUartRecv.aucRxBuf[(g_tDCUartRecv.uiByteNum - 1)]);
										
										/*����У���*/
										usCalCheckSum = XorCheckSum16((uint16_t *)&g_tDCUartRecv.aucRxBuf[6], ((g_tDCUartRecv.usPackLen + 1 - 2) / 2));
										
										/*�ж�У���*/
										if(usPJCheckSum == usCalCheckSum)
										{				
											g_tDCUartRecv.usCheckSum = usPJCheckSum;//��¼У���
											
											/*��Ϣ��ֵ������ָ��ṹ��*/
											g_tCXUartTimeBD.usPackHeader	= g_tDCUartRecv.usPackHeader;
											g_tCXUartTimeBD.usPackSeq		= g_tDCUartRecv.usPackSeq;
											g_tCXUartTimeBD.ePackNoType		= g_tDCUartRecv.ePackNoType;												
											g_tCXUartTimeBD.usPackNo		= g_tDCUartRecv.usPackNo;
											g_tCXUartTimeBD.usPackLen		= g_tDCUartRecv.usPackLen;
											g_tCXUartTimeBD.usPackID		= g_tDCUartRecv.usPackID; 
											g_tCXUartTimeBD.usCheckSum		= g_tDCUartRecv.usCheckSum; 

											memcpy((void *)&g_tCXUartTimeBD.aucTimeData[0], (void *)&g_tDCUartRecv.aucRxBuf[8], (g_tDCUartRecv.usPackLen + 1 - 4));//����ң��Ӧ������
											
											/*������ɣ���λ����*/
											UART_Reset(eUartObject);
											
											/*�������ݴ���*/
											UART_Data_Process(eCX_TimeBD);
										}
										else
										{			
											g_tCXUartState.ucRecvErrorCmdCnt = g_tCXUartState.ucRecvErrorCmdCnt + 1;//����ָ�����+1											
											UART_Reset(eUartObject);//��У������󣬰ѻ��渴λ
										}
									}
									else
									{
										;//ֻ�洢����������
									}
								}
								else
								{
									;//�ֽ�7����������
								}
							}
							else
							{
								;//�ֽ�5����������
							}
						}
						else
						{
							;//�ֽ�3����������
						}	
					}
					else
					{
						;//��������
					}
				}
			}
		}
    }
}
	

/************************************************************************
    ��  ��  ��: UART_YK_Cmd_Process
    ����˵��: UARTָ���
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void CX_UART_YK_Cmd_Process(E_UART_DATA_TYPE eUartDataType)
{
	T_CX_UART_YK_CMD 	tUartYKCmd;
	uint32_t 			uiUartTxDataAddr= 0;	//���ڽ�������fifo��ַ
	uint16_t 			usCmdType 		= 0;	//ָ������
	uint8_t				ucSubCmdType 	= 0;	//ָ��������
	uint32_t			uiCmdData		= 0;	//ָ������
	
	if(eUartDataType == eCX_YKCmd)
	{
		uiUartTxDataAddr = UART_CX_TXDATA_ADDR_DEBUG;
		memcpy((void *)&tUartYKCmd, (void *)&g_tCXUartYKCmd, sizeof(g_tCXUartYKCmd));
	}
	else if(eUartDataType == eDC_YKCmd)
	{
		uiUartTxDataAddr = UART_DC_TXDATA_ADDR_DEBUG;
		memcpy((void *)&tUartYKCmd, (void *)&g_tDCUartYKCmd, sizeof(g_tDCUartYKCmd));
	}
	else
	{
		return;
	}
	
	usCmdType = tUartYKCmd.usCmdType;
	
	switch(usCmdType)
	{
		/****************************************�������****************************************/
		case 0xA140://�ŵ翪�ؽ�ͨ
			memcpy((void *)&g_tCXUartYKCmdState.aucK521[0], (void *)&tUartYKCmd.aucCmdData[0], 4);		
			FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
			//״̬�жϣ�������ң���н����ж���ӣ�����ʱ��ֱ�Ӹ���ͨ״̬
			g_tCXUartState.ucDischargeSwitch = 0xA0;
			break;
		case 0xA141://�ŵ翪�ضϿ�
			memcpy((void *)&g_tCXUartYKCmdState.aucK522[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(XDC_CLOSE_ADDR) = 0x3333;
			//״̬�жϣ�������ң���н����ж���ӣ�����ʱ��ֱ�Ӹ��Ͽ�״̬
			g_tCXUartState.ucDischargeSwitch = 0x0A;
			break;
		case 0xA142://�ƽ����翪�ؽ�ͨ				
			memcpy((void *)&g_tCXUartYKCmdState.aucK523[0], (void *)&tUartYKCmd.aucCmdData[0], 4);		
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0055;		
			break;
		case 0xA143://�ƽ����翪�ضϿ�		
			memcpy((void *)&g_tCXUartYKCmdState.aucK524[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x00AA;			
			break;
		case 0xA144://����������翪�ؽ�ͨ
			memcpy((void *)&g_tCXUartYKCmdState.aucK525[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0255;		
			break;
		case 0xA145://����������翪�ضϿ�			
			memcpy((void *)&g_tCXUartYKCmdState.aucK526[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x02AA;
			break;
		case 0xA146://ͨ�Ż����翪�ؽ�ͨ
			memcpy((void *)&g_tCXUartYKCmdState.aucK527[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0355;			
			break;
		case 0xA147://ͨ�Ż����翪�ضϿ�
			memcpy((void *)&g_tCXUartYKCmdState.aucK528[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x03AA;
			break;
		case 0xA148://�������翪�ؽ�ͨ							
			memcpy((void *)&g_tCXUartYKCmdState.aucK529[0], (void *)&tUartYKCmd.aucCmdData[0], 4);		
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0455;	
			break;
		case 0xA149://�������翪�ضϿ�				
			memcpy((void *)&g_tCXUartYKCmdState.aucK5210[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x04AA;
			break;
		case 0xA14A://���鹩�翪�ؽ�ͨ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5211[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0555;	
			break;
		case 0xA14B://���鹩�翪�ضϿ�			
			memcpy((void *)&g_tCXUartYKCmdState.aucK5212[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x05AA;
			break;
		case 0xA14C://GNSS���翪�ؽ�ͨ						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5213[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0755;
			break;
		case 0xA14D://GNSS���翪�ضϿ�						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5214[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x07AA;
			break;
		case 0xA201:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5215[0], (void *)&tUartYKCmd.aucCmdData[0], 4);			
			break;
		case 0xA202:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5216[0], (void *)&tUartYKCmd.aucCmdData[0], 4);			
			break;
		case 0xA203://�Լ�						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5217[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			g_tCXUartState.ucSelfCheckState = 0x40;
			g_tCXUartState.ucSateWorkMode = 0x08;
			break;
		case 0xA204:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5218[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			
			break;
		case 0xA205:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5219[0], (void *)&tUartYKCmd.aucCmdData[0], 4);		
			break;
		case 0xA206:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5220[0], (void *)&tUartYKCmd.aucCmdData[0], 4);			
			break;
		case 0xA207:						
			memcpy((void *)&g_tCXUartYKCmdState.aucK5221[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA208:
			ucSubCmdType = tUartYKCmd.aucCmdData[0]; 
			switch(ucSubCmdType)
			{
				case 0x01:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5222[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x02:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5223[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0A:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5224[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0B:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5225[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA5A5://ת�ڵ�ָ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			if(uiCmdData == 0x5555AAAA)
			{
				memcpy((void *)&g_tCXUartYKCmdState.aucK5226[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
				//�������ؿ���
				FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
				//״̬�жϣ�������ң���н����ж���ӣ�����ʱ��ֱ�Ӹ���״̬
				g_tCXUartState.ucDischargeSwitch = 0xA0;
			}
			break;
		case 0x9A9A://ת�ڵ�ɹ�ָ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);;
			if(uiCmdData == 0x1111AAAA)
			{
				memcpy((void *)&g_tCXUartYKCmdState.aucK5227[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
				/*ת��ɹ�������C9C9*/
				FPGAUart_SendByte(uiUartTxDataAddr, 0xC9);
				FPGAUart_SendByte(uiUartTxDataAddr, 0xC9);
				
				delay_1ms(10);
				
				//����CX422����
				FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0BAA;
				//��״̬�жϣ�ֱ�Ӹ��Ͽ�״̬
				g_tCXUartState.uc422State = 0x03;//�Ͽ� 
			}
			break;
		case 0xA9A9://�����ã���CX422
			FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0B55;//��422
			//��״̬�жϣ�ֱ�Ӹ���ͨ״̬
			g_tCXUartState.uc422State = 0x00;//��ͨ
			break;
		/****************************************ͨ�Ż����****************************************/
		case 0xA402://ͨ�Ż��������
			memcpy((void *)&g_tCXUartYKCmdState.aucK5228[0], (void *)&tUartYKCmd.aucCmdData[0], 4);	
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x02;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		case 0xA403://ͨ�Ż��������
			memcpy((void *)&g_tCXUartYKCmdState.aucK5229[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x03;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;	
			break;
		case 0xA404://ͨ�Ż�����͹���ģʽ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5230[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x04;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		case 0xA405://ͨ�Ż�����߹���ģʽ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5231[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x05;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		case 0xA406://ͨ�Ż�FPGA��λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5232[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x06;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		case 0xA407://ͨ�Ż�CANA���߸�λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5233[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x07;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		case 0xA408://ͨ�Ż�CANB���߸�λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5234[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_TXJYKCMD.aucData[0] 			= 0x08;
			g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
			break;
		/****************************************���ȴ����****************************************/
		case 0xA301:
			memcpy((void *)&g_tCXUartYKCmdState.aucK5235[0], (void *)&tUartYKCmd.aucCmdData[0], 4);			
			break;
		case 0xA302: 
			memcpy((void *)&g_tCXUartYKCmdState.aucK5236[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA303:
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0x0155AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5237[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0255AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5238[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0355AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5239[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0455AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5240[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0555AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5241[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0655AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5242[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0755AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5243[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0855AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5244[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0955AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5245[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0A55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5246[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0B55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5247[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0C55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5248[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0D55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5249[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0E55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5250[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0F55AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5251[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1055AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5252[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1155AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5253[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1255AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5254[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1355AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5255[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1455AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5256[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x1555AAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5257[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA304:
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0x01AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5258[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x02AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5259[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x03AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5260[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x04AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5261[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x05AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5262[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x06AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5263[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x07AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5264[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x08AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5265[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x09AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5266[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0AAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5267[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0BAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5268[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0CAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5269[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0DAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5270[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0EAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5271[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0FAAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5272[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x10AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5273[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x11AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5274[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x12AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5275[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x13AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5276[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x14AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5277[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x15AAAAAA:
					memcpy((void *)&g_tCXUartYKCmdState.aucK5278[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		/****************************************�������****************************************/
		case 0xA501://���鴦����������λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5279[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_GMYKCMD.aucData[0] = 0x01;
			memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
			g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
			g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
			g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;
			break;
		case 0xA502://CANA���߸�λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5280[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_GMYKCMD.aucData[0] = 0x02;
			memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
			g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
			g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
			g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;
			break;
		case 0xA503://CANB���߸�λ
			memcpy((void *)&g_tCXUartYKCmdState.aucK5281[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_GMYKCMD.aucData[0] = 0x03;
			memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
			g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
			g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
			g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;
			break;
		case 0xA504://�������
			memcpy((void *)&g_tCXUartYKCmdState.aucK5282[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			g_tCanSend_GMYKCMD.aucData[0] = 0x04;
			memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
			g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
			g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
			g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;
			break;
		case 0xA505://�����Լ�
			memcpy((void *)&g_tCXUartYKCmdState.aucK5283[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA506://����ǰ����ͼ��ش�ָ��
			memcpy((void *)&g_tCXUartYKCmdState.aucK5284[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA507://��ʼͼ��ش�ָ��
			memcpy((void *)&g_tCXUartYKCmdState.aucK5285[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA508://���鴦�����������дָ��
			memcpy((void *)&g_tCXUartYKCmdState.aucK5286[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA509://ֹͣͼ��ش�ָ��
			memcpy((void *)&g_tCXUartYKCmdState.aucK5287[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA50A://���鴦������Լ�ָ��
			memcpy((void *)&g_tCXUartYKCmdState.aucK5288[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
			break;
		case 0xA50B://�������ģʽѡ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://�����ģʽ1��
					memcpy((void *)&g_tCXUartYKCmdState.aucK5289[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://�����ģʽ2��
					memcpy((void *)&g_tCXUartYKCmdState.aucK5290[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xCCAAAAAA://�����ģʽ3��
					memcpy((void *)&g_tCXUartYKCmdState.aucK5291[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xDDAAAAAA://�����ģʽ4��
					memcpy((void *)&g_tCXUartYKCmdState.aucK5292[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}				
			break;
		case 0xA50C://����ع�ģʽ
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://���ֶ���
					memcpy((void *)&g_tCXUartYKCmdState.aucK5293[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://���Զ���
					memcpy((void *)&g_tCXUartYKCmdState.aucK5294[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}			
			break;
		case 0xA50F://���֡Ƶ
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0x01AAAAAA://1HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK5295[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x02AAAAAA://2HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK5296[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x03AAAAAA://3HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK5297[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x04AAAAAA://4HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK5298[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x05AAAAAA://5HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK5299[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x06AAAAAA://6HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52100[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x07AAAAAA://7HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52101[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x08AAAAAA://8HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52102[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x09AAAAAA://9HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52103[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0AAAAAAA://10HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52104[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0BAAAAAA://11HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52105[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0CAAAAAA://12HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52106[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0DAAAAAA://13HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52107[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0EAAAAAA://14HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52108[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0FAAAAAA://15HZ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52109[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA510://�������ģʽ
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://������ģʽ��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52110[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://��֡Ƶģʽ��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52111[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA511://���ͼ��ʹ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xCAAAAAAA://����ʼ��ͼ��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52112[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xCFAAAAAA://��ֹͣ��ͼ��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52113[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA512://��������洢ʹ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://����ʼ�洢��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52114[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://��ֹͣ�洢��
					memcpy((void *)&g_tCXUartYKCmdState.aucK52115[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA514://�������ػ�ָ��
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://������
					memcpy((void *)&g_tCXUartYKCmdState.aucK52116[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://���أ�
					memcpy((void *)&g_tCXUartYKCmdState.aucK52117[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA515://������Ƶ
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0x01AAAAAA://1Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52118[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x02AAAAAA://2Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52119[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x03AAAAAA://3Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52120[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x04AAAAAA://4Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52121[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x05AAAAAA://5Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52122[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x06AAAAAA://6Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52123[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x07AAAAAA://7Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52124[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x08AAAAAA://8Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52125[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x09AAAAAA://9Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52126[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0x0AAAAAAA://10Hz
					memcpy((void *)&g_tCXUartYKCmdState.aucK52127[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA516://��������״̬
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://��������ࣩ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52128[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://���Զ���ࣩ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52129[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xFFAAAAAA://��ֹͣ��ࣩ
					memcpy((void *)&g_tCXUartYKCmdState.aucK52130[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		case 0xA518://���鴦������㲥���ݿ���
			uiCmdData = u8_2_u32(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1], tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3]);
			switch(uiCmdData)
			{
				case 0xAAAAAAAA://��������
					memcpy((void *)&g_tCXUartYKCmdState.aucK52131[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				case 0xBBAAAAAA://���رգ�
					memcpy((void *)&g_tCXUartYKCmdState.aucK52132[0], (void *)&tUartYKCmd.aucCmdData[0], 4);
					break;
				default:
					break;
			}
			break;
		/****************************************����ע�����****************************************/
		case 0x0033://��׼������Ч���� K52279
			g_tCXUartStdTransferData.usGPSWeek 		= u8_2_u16(tUartYKCmd.aucCmdData[0], tUartYKCmd.aucCmdData[1]);
			g_tCXUartStdTransferData.uiGPSSecond 	= u8_2_u32(tUartYKCmd.aucCmdData[2], tUartYKCmd.aucCmdData[3], tUartYKCmd.aucCmdData[4], tUartYKCmd.aucCmdData[5]);
			g_tCXUartStdTransferData.fSateJ2000Rx 	= u8_2_flt(tUartYKCmd.aucCmdData[6], tUartYKCmd.aucCmdData[7], tUartYKCmd.aucCmdData[8], tUartYKCmd.aucCmdData[9]);
			g_tCXUartStdTransferData.fSateJ2000Ry 	= u8_2_flt(tUartYKCmd.aucCmdData[10], tUartYKCmd.aucCmdData[11], tUartYKCmd.aucCmdData[12], tUartYKCmd.aucCmdData[13]);
			g_tCXUartStdTransferData.fSateJ2000Rz 	= u8_2_flt(tUartYKCmd.aucCmdData[14], tUartYKCmd.aucCmdData[15], tUartYKCmd.aucCmdData[16], tUartYKCmd.aucCmdData[17]);
			g_tCXUartStdTransferData.fSateJ2000Vx 	= u8_2_flt(tUartYKCmd.aucCmdData[18], tUartYKCmd.aucCmdData[19], tUartYKCmd.aucCmdData[20], tUartYKCmd.aucCmdData[21]);
			g_tCXUartStdTransferData.fSateJ2000Vy 	= u8_2_flt(tUartYKCmd.aucCmdData[22], tUartYKCmd.aucCmdData[23], tUartYKCmd.aucCmdData[24], tUartYKCmd.aucCmdData[25]);
			g_tCXUartStdTransferData.fSateJ2000Vz 	= u8_2_flt(tUartYKCmd.aucCmdData[26], tUartYKCmd.aucCmdData[27], tUartYKCmd.aucCmdData[28], tUartYKCmd.aucCmdData[29]);
			break;	
		case 0x0077://�б�ѩ�����ϵ�� K52282
			g_tCXUartQBXFcoeff.adTimeDur[0]			= u8_2_dbl(&tUartYKCmd.aucCmdData[0]);
			g_tCXUartQBXFcoeff.adTimeDur[1]			= u8_2_dbl(&tUartYKCmd.aucCmdData[8]);
			g_tCXUartQBXFcoeff.iOrder				= u8_2_u32(tUartYKCmd.aucCmdData[16], tUartYKCmd.aucCmdData[17], tUartYKCmd.aucCmdData[18], tUartYKCmd.aucCmdData[19]);
			g_tCXUartQBXFcoeff.iSegNum				= u8_2_u32(tUartYKCmd.aucCmdData[20], tUartYKCmd.aucCmdData[21], tUartYKCmd.aucCmdData[22], tUartYKCmd.aucCmdData[23]);
			g_tCXUartQBXFcoeff.dSegDur				= u8_2_dbl(&tUartYKCmd.aucCmdData[24]);
			g_tCXUartQBXFcoeff.afTimeArray[0]		= u8_2_flt(tUartYKCmd.aucCmdData[32], tUartYKCmd.aucCmdData[33], tUartYKCmd.aucCmdData[34], tUartYKCmd.aucCmdData[35]);
			g_tCXUartQBXFcoeff.afTimeArray[1]		= u8_2_flt(tUartYKCmd.aucCmdData[36], tUartYKCmd.aucCmdData[37], tUartYKCmd.aucCmdData[38], tUartYKCmd.aucCmdData[39]);
			g_tCXUartQBXFcoeff.afTimeArray[2]		= u8_2_flt(tUartYKCmd.aucCmdData[40], tUartYKCmd.aucCmdData[41], tUartYKCmd.aucCmdData[42], tUartYKCmd.aucCmdData[43]);
			g_tCXUartQBXFcoeff.afTimeArray[3]		= u8_2_flt(tUartYKCmd.aucCmdData[44], tUartYKCmd.aucCmdData[45], tUartYKCmd.aucCmdData[46], tUartYKCmd.aucCmdData[47]);
			g_tCXUartQBXFcoeff.afTimeArray[4]		= u8_2_flt(tUartYKCmd.aucCmdData[48], tUartYKCmd.aucCmdData[49], tUartYKCmd.aucCmdData[50], tUartYKCmd.aucCmdData[51]);
			break;
		case 0x0066://ȫ����������� K52281
			memcpy(&g_tCXUartTaskParam.aucParam[0], &tUartYKCmd.aucCmdData[0], sizeof(g_tCXUartTaskParam));
			break;
		default :
			break;
	}
}

/************************************************************************
    ��  ��  ��: UART_YC_Request_Process
    ����˵��: UARTң��������
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void CX_UART_YC_Request_Process(E_UART_DATA_TYPE eUartDataType)
{	
	uint16_t	usXorCheckSum = 0;
	
	/*ң���������*/
	g_tCXUartYC.aucW0W1_PackHeader[0] 	= 0x0B;
	g_tCXUartYC.aucW0W1_PackHeader[1] 	= 0xC1;
	g_tCXUartYC.aucW2W3_PackSeq[0] 		= u16_2_u8(g_tCXUartState.usYCSeq, 1);
	g_tCXUartYC.aucW2W3_PackSeq[1] 		= u16_2_u8(g_tCXUartState.usYCSeq, 0);
	g_tCXUartYC.aucW4W5_PackLen[0]		= 0x00;
	g_tCXUartYC.aucW4W5_PackLen[1]		= 0x39;
	g_tCXUartYC.aucW6W11_HML[0]			= u16_2_u8(g_tCXUartState.usH, 1);
	g_tCXUartYC.aucW6W11_HML[1]			= u16_2_u8(g_tCXUartState.usH, 0);
	g_tCXUartYC.aucW6W11_HML[2]			= u16_2_u8(g_tCXUartState.usM, 1);
	g_tCXUartYC.aucW6W11_HML[3]			= u16_2_u8(g_tCXUartState.usM, 0);
	g_tCXUartYC.aucW6W11_HML[4]			= u16_2_u8(g_tCXUartState.usL, 1);
	g_tCXUartYC.aucW6W11_HML[5]			= u16_2_u8(g_tCXUartState.usL, 0); 
	g_tCXUartYC.ucW16_RecvCmdCnt		= g_tCXUartState.ucRecvCmdCnt;
	g_tCXUartYC.ucW17_SateWorkMode		= g_tCXUartState.ucSateWorkMode;//���ǹ���ģʽ��0000
	g_tCXUartYC.aucW22W23_[0]			= g_ZJState.ucSecondCnt + 1;
	g_tCXUartYC.ucW26_Reserved_Timing 	= (g_tCXUartState.uc422State << 6);
	g_tCXUartYC.ucW28_					= g_tCXUartState.ucSelfCheckState;//����״̬
	g_tCXUartYC.ucW41_Signal			= g_tTXJYC[2].aucTXJYC[0];
	g_tCXUartYC.ucW42_					= ((g_tTXJYC[2].aucTXJYC[7]&0x0F) << 4) | (g_tTXJYC[2].aucTXJYC[7] & 0x0F);
	g_tCXUartYC.ucW47_HGP				= g_tGMYC[2].aucGMYC[13];
	g_tCXUartYC.ucW48_ZHState			= g_tGMYC[2].aucGMYC[16];
	g_tCXUartYC.ucW49_ZDBState			= g_tGMYC[2].aucGMYC[17];
	g_tCXUartYC.ucW50_CameraState		= g_tGMYC[2].aucGMYC[29];
	g_tCXUartYC.ucW51_TargetNum			= g_tGMYC[2].aucGMYC[37];
	g_tCXUartYC.ucW52_Light				= g_tGMYC[2].aucGMYC[38];
	g_tCXUartYC.ucW53_TargetDis			= g_tGMYC[2].aucGMYC[68];
	g_tCXUartYC.ucW54_DMEState			= g_tGMYC[2].aucGMYC[99];
	g_tCXUartYC.aucW55W61_FillParam[4] 	= g_tTXJUartState.ucRecvCmdCnt;
	g_tCXUartYC.aucW55W61_FillParam[5] 	= g_tTXJUartState.ucProcessCmdCnt;
	g_tCXUartYC.aucW55W61_FillParam[6] 	= g_tTXJUartState.ucRecvErrorCmdCnt;
	
	usXorCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartYC.aucW6W11_HML[0], 28);
	g_tCXUartYC.aucW62W63_CheckSum[0]	= u16_2_u8(usXorCheckSum, 0);
	g_tCXUartYC.aucW62W63_CheckSum[1]	= u16_2_u8(usXorCheckSum, 1);
	
	if(eUartDataType == eCX_YCRequest)
	{
		g_tCXUartState.ucYCSeq1 = 0x03;
		g_tCXUartState.usYCSeq2 = g_tCXUartState.usYCSeq2 + 1;
		g_tCXUartState.usYCSeq2 = g_tCXUartState.usYCSeq2 % 16384;
		
		CSL_FINSR(g_tCXUartState.usYCSeq, 15, 14, g_tCXUartState.ucYCSeq1);
		CSL_FINSR(g_tCXUartState.usYCSeq, 13, 0, g_tCXUartState.usYCSeq2);
		
		CSL_FINSR(g_tGMYC[2].aucGMYC[16],7,7,~CSL_FEXTR(g_tGMYC[2].aucGMYC[16], 7, 7));
				
		/*��CX����ң������*/
		FPGAUart_SendArray(UART_CX_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartYC, sizeof(g_tCXUartYC));
	}
	else if(eUartDataType == eDC_YCRequest)
	{
		/*��زⷢ��ң������*/
		FPGAUart_SendArray(UART_DC_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartYC, sizeof(g_tCXUartYC));
	}
	else
	{
		;
	}
	


}

/************************************************************************
    ��  ��  ��: UART_SC_Request_Process
    ����˵��: UART����������
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void CX_UART_SC_Request_Process(E_UART_DATA_TYPE eUartDataType)
{
			uint16_t	usXorCheckSum 	= 0;
			Uint8 		ucSeparationt 	= 0;
	static	Uint8 		sucSendFlag		= 0;
		
	if(sucSendFlag == 0)
	{
		
		if(FPGA_REG_RW_2BYTE(XJFL_RD1_ADDR) == 0x5555)
		{
			CSL_FINSR(ucSeparationt, 7, 4, 0x3);
		}
		else if(FPGA_REG_RW_2BYTE(XJFL_RD1_ADDR) == 0xAAAA)
		{
			CSL_FINSR(ucSeparationt, 7, 4, 0xC);
		}
		
		if(FPGA_REG_RW_2BYTE(XJFL_RD2_ADDR) == 0x5555)
		{
			CSL_FINSR(ucSeparationt, 3, 0, 0x3);
		}
		else if(FPGA_REG_RW_2BYTE(XJFL_RD2_ADDR) == 0xAAAA)
		{
			CSL_FINSR(ucSeparationt, 3, 0, 0xC);
		}
		
		/*����������1���*/
		g_tCXUartSC1.aucW0W1_PackHeader[0] 		= 0xEB;
		g_tCXUartSC1.aucW0W1_PackHeader[1] 		= 0x90;
		g_tCXUartSC1.aucW2W3_PackId[0] 			= 0x0B;
		g_tCXUartSC1.aucW2W3_PackId[1] 			= 0xC2;
		g_tCXUartSC1.aucW4W5_PackSeq[0]			= u16_2_u8(g_tCXUartState.usSCSeq, 1); 
		g_tCXUartSC1.aucW4W5_PackSeq[1]			= u16_2_u8(g_tCXUartState.usSCSeq, 0); 
		g_tCXUartSC1.aucW6W7_PackLen[0]			= 0x00;
		g_tCXUartSC1.aucW6W7_PackLen[1]			= 0xF7;
		g_tCXUartSC1.aucW8W13_HML[0]			= u16_2_u8(g_tCXUartState.usH, 1);
		g_tCXUartSC1.aucW8W13_HML[1]			= u16_2_u8(g_tCXUartState.usH, 0);
		g_tCXUartSC1.aucW8W13_HML[2]			= u16_2_u8(g_tCXUartState.usM, 1);
		g_tCXUartSC1.aucW8W13_HML[3]			= u16_2_u8(g_tCXUartState.usM, 0);
		g_tCXUartSC1.aucW8W13_HML[4]			= u16_2_u8(g_tCXUartState.usL, 1);
		g_tCXUartSC1.aucW8W13_HML[5]			= u16_2_u8(g_tCXUartState.usL, 0);
		g_tCXUartSC1.ucW14_SCRequestCnt			= g_tCXUartState.ucSCRequestCnt;//�����������
		g_tCXUartSC1.ucW15_YCRequestCnt			= g_tCXUartState.ucYCRequestCnt;//ң���������
		g_tCXUartSC1.ucW16_RecvCmdCnt			= g_tCXUartState.ucRecvCmdCnt;//����ָ���������ȷָ�����+����ָ�����
		g_tCXUartSC1.ucW17_ProcessCmdCnt		= g_tCXUartState.ucProcessCmdCnt;//��ȷִ��ָ���������ȷָ�����
		g_tCXUartSC1.ucW18_RecvErrorCmdCnt		= 0; 
		g_tCXUartSC1.aucW48W253_[0]				= g_tCXUartState.ucDischargeSwitch;//�ŵ翪��״̬
		g_tCXUartSC1.ucW47_Separationt			= ucSeparationt;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartSC1.aucW8W13_HML[0], 123);
		g_tCXUartSC1.aucW254W255_CheckSum[0]	= u16_2_u8(usXorCheckSum, 0);
		g_tCXUartSC1.aucW254W255_CheckSum[1]	= u16_2_u8(usXorCheckSum, 1);
		
		if(eUartDataType == eCX_SCRequest)
		{
			g_tCXUartState.ucSCSeq1 = 0x02;
			g_tCXUartState.usSCSeq2 = g_tCXUartState.usSCSeq2 + 1;
			g_tCXUartState.usSCSeq2 = g_tCXUartState.usSCSeq2 % 16384; 
			
			CSL_FINSR(g_tCXUartState.usSCSeq, 15, 14, g_tCXUartState.ucSCSeq1);
			CSL_FINSR(g_tCXUartState.usSCSeq, 13, 0, g_tCXUartState.usSCSeq2);
			/*������������*/
			FPGAUart_SendArray(UART_CX_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartSC1, sizeof(g_tCXUartSC1));
		}
		else if(eUartDataType == eDC_SCRequest)
		{
			/*������������*/
			FPGAUart_SendArray(UART_DC_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartSC1, sizeof(g_tCXUartSC1));
		}
		else
		{
			;
		}	
		
		sucSendFlag = 1;
		
		return;
	}
	else
	{
		/*����������2���*/
		g_tCXUartSC2.aucW0W1_PackHeader[0] 		= 0xEB;
		g_tCXUartSC2.aucW0W1_PackHeader[1] 		= 0x90;
		g_tCXUartSC2.aucW2W3_PackId[0] 			= 0x0B;
		g_tCXUartSC2.aucW2W3_PackId[1] 			= 0xC2;
		g_tCXUartSC2.aucW4W5_PackSeq[0]			= u16_2_u8(g_tCXUartState.usSCSeq, 1); 
		g_tCXUartSC2.aucW4W5_PackSeq[1]			= u16_2_u8(g_tCXUartState.usSCSeq, 0); 
		g_tCXUartSC2.aucW6W7_PackLen[0]			= 0x00;
		g_tCXUartSC2.aucW6W7_PackLen[1]			= 0xF7;
		g_tCXUartSC2.aucW8W13_HML[0]			= u16_2_u8(g_tCXUartState.usH, 1);
		g_tCXUartSC2.aucW8W13_HML[1]			= u16_2_u8(g_tCXUartState.usH, 0);
		g_tCXUartSC2.aucW8W13_HML[2]			= u16_2_u8(g_tCXUartState.usM, 1);
		g_tCXUartSC2.aucW8W13_HML[3]			= u16_2_u8(g_tCXUartState.usM, 0);
		g_tCXUartSC2.aucW8W13_HML[4]			= u16_2_u8(g_tCXUartState.usL, 1);
		g_tCXUartSC2.aucW8W13_HML[5]			= u16_2_u8(g_tCXUartState.usL, 0);
		g_tCXUartSC2.ucW14W253_[107]			= g_ZJState.ucSecondCnt;
		g_tCXUartSC2.ucW14W253_[24]				= g_ZJState.ucSecondCnt;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tCXUartSC2.aucW8W13_HML[0], 123);
		g_tCXUartSC2.aucW254W255_CheckSum[0]	= u16_2_u8(usXorCheckSum, 0);
		g_tCXUartSC2.aucW254W255_CheckSum[1]	= u16_2_u8(usXorCheckSum, 1);
		
		if(eUartDataType == eCX_SCRequest)
		{
			g_tCXUartState.ucSCSeq1 = 0x01;
			g_tCXUartState.usSCSeq2 = g_tCXUartState.usSCSeq2 + 1;
			g_tCXUartState.usSCSeq2 = g_tCXUartState.usSCSeq2 % 16384;
			
			CSL_FINSR(g_tCXUartState.usSCSeq, 15, 14, g_tCXUartState.ucSCSeq1); 
			CSL_FINSR(g_tCXUartState.usSCSeq, 13, 0, g_tCXUartState.usSCSeq2);
			/*������������*/
			FPGAUart_SendArray(UART_CX_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartSC2, sizeof(g_tCXUartSC2));
		}
		else if(eUartDataType == eDC_SCRequest)
		{
			/*������������*/
			FPGAUart_SendArray(UART_DC_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tCXUartSC2, sizeof(g_tCXUartSC2));
		}
		else
		{
			;
		}	
		
		sucSendFlag = 0;
		
		return;
	}
	
	
}

/************************************************************************
    ��  ��  ��: TXJ_UART_Data_Analysis
    ����˵��: UART�����ֽ����ݣ�У�����
    �������: ��
    �������:
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void TXJ_UART_Data_Analysis(E_UART_OBJECT eUartObject)
{
    uint32_t uiUartRxDataAddr = 0;  // ���ڽ�������FIFO��ַ
    uint16_t usDataIdx = 0;         // ���ڽ�����������
    uint16_t usRxFifoDataNum = 0;   // ���ڽ������ݸ���
    uint8_t ucUartByte = 0;         // ���ڽ��յ��ĵ��ֽ�����
    uint16_t usPJPackHead = 0;      // ƴ�ӵõ��İ�ͷ
    uint16_t usPJDataLen = 0;       // ƴ�ӵõ������ݳ���
    UART_STATE currentState = STATE_FIND_HEADER;  // ��ʼ״̬
    static UART_STATE nextState = STATE_FIND_HEADER;

    usRxFifoDataNum = Is_FPGA_Uart_RxFifo_Empty(eUartObject);

    /* ��ȡ���ڽ���fifo�е����ݸ��� */
    usRxFifoDataNum = Is_FPGA_Uart_RxFifo_Empty(eUartObject);

    switch (eUartObject)
    {
        case eCX:
					uiUartRxDataAddr = UART_CX_RXDATA_ADDR_DEBUG;
					break;
        case eTXJ:
					uiUartRxDataAddr = UART_TXJ_RXDATA_ADDR_DEBUG;
					break;
        case eDC:
					uiUartRxDataAddr = UART_DC_RXDATA_ADDR_DEBUG;
					break;
        case eGZ:
					uiUartRxDataAddr = UART_GZ_RXDATA_ADDR_DEBUG;
					break;
        default:
					break;
    }

    if (usRxFifoDataNum)
    {
        for (usDataIdx = 0; usDataIdx < usRxFifoDataNum; usDataIdx++)
        {
            ucUartByte = FPGA_REG_RW_1BYTE(uiUartRxDataAddr);

            // �����յ����ֽڴ洢������
            g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.uiByteNum++] = ucUartByte;

            // ���ݵ�ǰ״̬���в�ͬ�Ĳ���
            switch (currentState)
            {
                case STATE_FIND_HEADER:
                    if (g_tTXJUartRecv.uiByteNum == 1)
                    {
                        if (g_tTXJUartRecv.aucRxBuf[0] != TXJ_UART_SZ_PACKHEAD_FIRST)
                        {
                            UART_Reset(eUartObject);
                        }
                    }
                    else if (g_tTXJUartRecv.uiByteNum >= 2)
                    {
                        usPJPackHead = u8_2_u16(g_tTXJUartRecv.aucRxBuf[0], g_tTXJUartRecv.aucRxBuf[1]);
                        if (usPJPackHead == TXJ_UART_SZ_PACKHEAD)
                        {
                            g_tTXJUartRecv.usPackHead = usPJPackHead;
                            nextState = STATE_PARSE_LENGTH;
                        }
                        else
                        {
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_LENGTH:
                    if (g_tTXJUartRecv.uiByteNum == 4)
                    {
                        usPJDataLen = u8_2_u16(g_tTXJUartRecv.aucRxBuf[2], g_tTXJUartRecv.aucRxBuf[3]);
                        if (usPJDataLen == 0x0015 || usPJDataLen == 0x0045 || usPJDataLen == 0x0085)
                        {
                            g_tTXJUartRecv.usDataLen = usPJDataLen;
                            nextState = STATE_PARSE_TYPE;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_TYPE:
                    if (g_tTXJUartRecv.uiByteNum == 5)
                    {
                        uint8_t ucDataType = g_tTXJUartRecv.aucRxBuf[4];
                        if (ucDataType == TXJ_UART_SZ_DATA_TYPE)
                        {
                            g_tTXJUartRecv.ucDataType = ucDataType;
                            nextState = STATE_PARSE_SYNC_WORD;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_SYNC_WORD:
                    if (g_tTXJUartRecv.uiByteNum == 7)
                    {
                        uint16_t usPJSateSynWord = u8_2_u16(g_tTXJUartRecv.aucRxBuf[5], g_tTXJUartRecv.aucRxBuf[6]);
                        if (usPJSateSynWord == TXJ_UART_YK_CMD_SATESYNWORD)
                        {
                            g_tTXJUartRecv.usSateSynWord = usPJSateSynWord;
                            nextState = STATE_PARSE_MODE;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_MODE:
                    if (g_tTXJUartRecv.uiByteNum == 8)
                    {
                        uint8_t ucModeWord = g_tTXJUartRecv.aucRxBuf[7];
                        switch (ucModeWord)
                        {
                            case 0xA5: g_tTXJUartRecv.ucPackLen = 16; break;
                            case 0x69: g_tTXJUartRecv.ucPackLen = 64; break;
                            case 0x3C: g_tTXJUartRecv.ucPackLen = 128; break;
                            default:
                                g_tTXJUartState.ucRecvErrorCmdCnt++;
                                UART_Reset(eUartObject);
                                return;
                        }
                        nextState = STATE_PARSE_DATA;
                    }
                    break;

                case STATE_PARSE_DATA:
                    if (g_tTXJUartRecv.uiByteNum == (g_tTXJUartRecv.ucPackLen + 5 + 5))
                    {
                        uint16_t usPJCRC = u8_2_u16(g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.uiByteNum - 2], g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.uiByteNum - 1]);
                        uint16_t usCalCRC = CRC16_2(&g_tTXJUartRecv.aucRxBuf[8], g_tTXJUartRecv.ucPackLen);
                        if (usCalCRC == usPJCRC)
                        {
                            g_tTXJUartRecv.usCRC = usCalCRC;
                            memcpy(g_tTXJUartYKCmd.aucCmdData, &g_tTXJUartRecv.aucRxBuf[9], g_tTXJUartRecv.ucPackLen - 1);
                            nextState = STATE_PARSE_CRC;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_CRC:
                    if (g_tTXJUartRecv.uiByteNum == (g_tTXJUartRecv.ucPackLen + 5 + 5 + 2))
                    {
                        uint16_t usPJCheckSum = u8_2_u16(g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.ucPackLen + 5 + 5], g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.ucPackLen + 5 + 5 + 1]);
                        uint16_t usCalCheckSum = AddCheckSum16(&g_tTXJUartRecv.aucRxBuf[2], g_tTXJUartRecv.ucPackLen + 7);
                        if (usCalCheckSum == usPJCheckSum)
                        {
                            nextState = STATE_PARSE_END;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                case STATE_PARSE_END:
                    if (g_tTXJUartRecv.uiByteNum == (g_tTXJUartRecv.ucPackLen + 5 + 5 + 4))
                    {
                        uint16_t usPJPackEnd = u8_2_u16(g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.ucPackLen + 5 + 5 + 2], g_tTXJUartRecv.aucRxBuf[g_tTXJUartRecv.ucPackLen + 5 + 5 + 3]);
                        if (usPJPackEnd == TXJ_UART_SZ_PACKEND)
                        {
                            UART_Reset(eUartObject);  // ��λ���壬�������
                            UART_Data_Process(eTXJ_YKCmd);
                            g_tTXJUartState.ucRecvCmdCnt++;
                        }
                        else
                        {
                            g_tTXJUartState.ucRecvErrorCmdCnt++;
                            UART_Reset(eUartObject);
                        }
                    }
                    break;

                default: break;
            }

            // ���µ�ǰ״̬
            currentState = nextState;
        }
    }
}


/************************************************************************
    ��  ��  ��: UART_YK_Cmd_Process
    ����˵��: UARTָ���
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void TXJ_UART_YK_Cmd_Process(void)
{
	uint32_t uiCmdCode = 0;
	
	switch(g_tTXJUartYKCmd.ucID)
	{
		case 0x00://���ϵ�������
			g_tTXJUartNavCmd.usGPSWeek 		= u8_2_u16(g_tTXJUartYKCmd.aucCmdData[2], g_tTXJUartYKCmd.aucCmdData[3]);
			g_tTXJUartNavCmd.uiGPSSecond 	= u8_2_u32(g_tTXJUartYKCmd.aucCmdData[4], g_tTXJUartYKCmd.aucCmdData[5], g_tTXJUartYKCmd.aucCmdData[6], g_tTXJUartYKCmd.aucCmdData[7]);
			g_tTXJUartNavCmd.fSateJ2000Rx 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[8], g_tTXJUartYKCmd.aucCmdData[9], g_tTXJUartYKCmd.aucCmdData[10], g_tTXJUartYKCmd.aucCmdData[11]);
			g_tTXJUartNavCmd.fSateJ2000Ry 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[12], g_tTXJUartYKCmd.aucCmdData[13], g_tTXJUartYKCmd.aucCmdData[14], g_tTXJUartYKCmd.aucCmdData[15]);
			g_tTXJUartNavCmd.fSateJ2000Rz 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[16], g_tTXJUartYKCmd.aucCmdData[17], g_tTXJUartYKCmd.aucCmdData[18], g_tTXJUartYKCmd.aucCmdData[19]);
			g_tTXJUartNavCmd.fSateJ2000Vx 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[20], g_tTXJUartYKCmd.aucCmdData[21], g_tTXJUartYKCmd.aucCmdData[22], g_tTXJUartYKCmd.aucCmdData[23]);
			g_tTXJUartNavCmd.fSateJ2000Vy 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[24], g_tTXJUartYKCmd.aucCmdData[25], g_tTXJUartYKCmd.aucCmdData[26], g_tTXJUartYKCmd.aucCmdData[27]);
			g_tTXJUartNavCmd.fSateJ2000Vz 	= u8_2_flt(g_tTXJUartYKCmd.aucCmdData[28], g_tTXJUartYKCmd.aucCmdData[29], g_tTXJUartYKCmd.aucCmdData[30], g_tTXJUartYKCmd.aucCmdData[31]);
			break;
		/*���񹩵�*/
		case 0xA1:
			uiCmdCode = u8_2_u32(0, g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2]);
			switch(uiCmdCode)
			{
				case 0x00000140://�ŵ翪�ؽ�ͨ
//					FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
					break;
				case 0x00000141://�ŵ翪�ضϿ�
//					FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
					break;
				case 0x00000242://�ƽ����翪�ؽ�ͨ
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0055;
					break;
				case 0x00000243://�ƽ����翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x00AA;
					break;
				case 0x00000244://����������翪�ؽ�ͨ
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0255;
					break;
				case 0x00000245://����������翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x02AA;
					break;
				case 0x00000246://ͨ�Ż����翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0355;
					break;
				case 0x00000247://ͨ�Ż����翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x03AA;
					break;
				case 0x0000026E://�������翪�ؽ�ͨ
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0455;
					break;
				case 0x0000026F://�������翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x04AA;
					break;
				case 0x00000270://���鹩�翪�ؽ�ͨ
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0555;
					break;
				case 0x00000271://���鹩�翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x05AA;
					break;
				case 0x00000272://GNSS���翪�ؽ�ͨ
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0755;
					break;
				case 0x00000273://GNSS���翪�ضϿ�
					FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x07AA;
					break;
				default:
					break;
			}		
			break;
		/*�������*/
		case 0xA2:
			uiCmdCode = u8_2_u32(0, g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2]);
			switch(uiCmdCode)
			{
				case 0x00000111:
					break;
				case 0x00000122:
					break;
				case 0x00000211:
					break;
				case 0x00000311://��Ҫң���´�ģʽ
					g_ZJState.ucWX_YCMode = 0;
					break;
				case 0x00000322://����ң���´�ģʽ
					g_ZJState.ucWX_YCMode = 1;
					break;
				case 0x00000611:
					break;
				case 0x00000622:
					break;
				case 0x0000080F:
					break;
				case 0x000008F0:
					break;
				case 0x00000A0A:
					break;
				case 0x00000A0B:
					break;
				default:
					break;
			}
			break;
		/*�·�ͨ�Ż�*/
		case 0xA3:
			uiCmdCode = u8_2_u32(0, g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2]);
			switch(uiCmdCode)
			{
				case 0x00000111://ͨ�Ż���
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0] 			= 0x02;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000122://ͨ�Ż���
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x03;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000211://ͨ�Ż�����͹���ģʽ
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x04;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000222://ͨ�Ż�����߹���ģʽ
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x05;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000311://ͨ�Ż�FPGA��λ
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x06;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000322://ͨ�Ż�CANA��λ
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x07;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000333://ͨ�Ż�CANB��λ
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x08;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000411://ͨ�Ż���ʼ����ͼ��
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x09;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x0000422://ͨ�Ż�ֹͣ����ͼ��
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0A;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000511://ͨ�Ż����ͼ�񻺴���
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0B;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000611://ͨ�Ż�OCָ��1
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0C;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000622://ͨ�Ż�OCָ��2
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0D;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000633://ͨ�Ż�OCָ��3
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0E;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				case 0x00000644://ͨ�Ż�OCָ��4
					//�·�ͨ�Ż�
					g_tCanSend_TXJYKCMD.aucData[0]			= 0x0F;
					g_tCanSendState[g_ZJState.ucTXJCan].tTXJYKCMD.uiFrameNum = 1;
					break;
				default:
					break;
			}
			break;
		case 0xA5://���ȴ�
			if(g_tTXJUartYKCmd.aucCmdData[1] == 0x02)
			{
				uiCmdCode = u8_2_u32(0, g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2]);
				switch(uiCmdCode)
				{
					default:
						break;
				}
			}
			else if(g_tTXJUartYKCmd.aucCmdData[1] == 0x03)
			{
				uiCmdCode = u8_2_u32(g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2], g_tTXJUartYKCmd.aucCmdData[4]);
				switch(uiCmdCode)
				{
					default:
						break;
				}
			}
			else
			{
				;
			}
			break;
		/*��������*/
		case 0xC1:
			uiCmdCode = u8_2_u32(g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2], g_tTXJUartYKCmd.aucCmdData[4]);
			break;
		/*�ؼ�ѹ��*/
		case 0xC2:
			uiCmdCode = u8_2_u32(g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2], g_tTXJUartYKCmd.aucCmdData[4]);
			break;
		/*�·�����*/
		case 0xC4:
			uiCmdCode = u8_2_u32(0, g_tTXJUartYKCmd.aucCmdData[0], g_tTXJUartYKCmd.aucCmdData[1], g_tTXJUartYKCmd.aucCmdData[2]);
			switch(uiCmdCode)
			{
				case 0x00001BAA://ʵʩZDB�ϵ�ָ��
					g_tCanSend_GMYKCMD.aucData[0] = 0x1B;
					memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
					g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
					g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
					g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;
					break;
				case 0x00001BBB://ʵʩZDB�µ�ָ��
					g_tCanSend_GMYKCMD.aucData[0] = 0x1B;
					memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
					g_tCanSend_GMYKCMD.aucData[6] = 0xBB;
					g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
					g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;		
					break;
				case 0x00000EAA://ʵʩZDB���ָ��
					g_tCanSend_GMYKCMD.aucData[0] = 0x14;
					memset(&g_tCanSend_GMYKCMD.aucData[1], 0, 5);
					g_tCanSend_GMYKCMD.aucData[6] = 0xAA;
					g_tCanSend_GMYKCMD.aucData[7] = AddCheckSum8(&g_tCanSend_GMYKCMD.aucData[0], 7);
					g_tCanSendState[g_ZJState.ucGMCan].tGMYKCMD.uiFrameNum = 1;			
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

/************************************************************************
    ��  ��  ��: TXJ_UART_YC_Pack
    ����˵��: ����ң�����
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void TXJ_UART_YC_Pack(void)
{
	uint32_t i = 0;
	uint16_t usXorCheckSum = 0;
	
	if(g_ZJState.ucWX_YCMode == 0)
	{
		/*��Ҫ��-----------------------------------------------------------------1*/
		g_tTXJUartYC_ZY[0].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_ZY[0].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_ZY[0].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_ZY[0].ucDataLenLow						= 0x80;
		g_tTXJUartYC_ZY[0].ucDataType						= 0x12;
				
		/*������Ҫң�������1���*/	
		g_tTXJUartYC_ZY[0].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_ZY[0].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_ZY[0].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_ZY[0].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_ZY[0].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
				
		g_tTXJUartYC_ZY[0].ucW33_PackNo						= 0x00;
		
		for(i = 0; i < 92; i++)
		{
			g_tTXJUartYC_ZY[0].aucW34W125_ValidData[i]		= 0xF0;
		}
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[91]			= 0x0F;
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[4]			= 0x50;
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[70]			= g_tTXJUartState.ucRecvCmdCnt;//����ָ�����
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[71]			= g_tTXJUartState.ucProcessCmdCnt;//��ȷִ��ָ�����
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[79]			= g_tTXJYC[2].aucTXJYC[0]; 
		g_tTXJUartYC_ZY[0].aucW34W125_ValidData[80]			= ((g_tTXJYC[2].aucTXJYC[7]&0x0F) << 4) | (g_tTXJYC[2].aucTXJYC[7] & 0x0F);
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_ZY[0].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_ZY[0].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0); 
		g_tTXJUartYC_ZY[0].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1); 
			
		g_tTXJUartYC_ZY[0].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_ZY[0].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_ZY[0].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_ZY[0].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_ZY[0].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_ZY[0].ucPackEndLow						= 0xD7;
			
		/*��Ҫ��-----------------------------------------------------------------2*/
		g_tTXJUartYC_ZY[1].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_ZY[1].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_ZY[1].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_ZY[1].ucDataLenLow						= 0x80;
		g_tTXJUartYC_ZY[1].ucDataType						= 0x12;
			
		/*������Ҫң�������2���*/	
		g_tTXJUartYC_ZY[1].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_ZY[1].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[2]			= 1;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[3]			= 1;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_ZY[1].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_ZY[1].ucW8_AuxiliaryHead				= 0x1F;//0b00011111;
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_ZY[1].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_ZY[1].ucW33_PackNo						= 0x01;
		
		for(i = 0; i < 92; i++)
		{
			g_tTXJUartYC_ZY[1].aucW34W125_ValidData[i]		= 0xF0;
		}
		
		g_tTXJUartYC_ZY[1].aucW34W125_ValidData[91]			= 0x0F;
				
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_ZY[1].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_ZY[1].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0); 
		g_tTXJUartYC_ZY[1].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1); 
				
		g_tTXJUartYC_ZY[1].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_ZY[1].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_ZY[1].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_ZY[1].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_ZY[1].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_ZY[1].ucPackEndLow						= 0xD7;
	}
	else
	{
		/*���̰�-----------------------------------------------------------------1*/
		g_tTXJUartYC_GC[0].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[0].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[0].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[0].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[0].ucDataType						= 0x12;
				
		/*���߹���ң�������1���*/	
		g_tTXJUartYC_GC[0].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[0].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[0].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[0].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[0].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[0].ucW33_PackNo						= 0x00;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[0].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[0].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[0].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
					 
		g_tTXJUartYC_GC[0].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[0].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[0].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[0].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[0].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[0].ucPackEndLow						= 0xD7;
		
		/*���̰�-----------------------------------------------------------------2*/
		g_tTXJUartYC_GC[1].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[1].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[1].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[1].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[1].ucDataType						= 0x12;
				
		/*���߹���ң�������1���*/	
		g_tTXJUartYC_GC[1].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[1].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[1].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[1].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[1].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[1].ucW33_PackNo						= 0x01;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[1].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[1].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[1].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
					 
		g_tTXJUartYC_GC[1].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[1].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[1].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[1].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[1].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[1].ucPackEndLow						= 0xD7;
		
		/*���̰�-----------------------------------------------------------------3*/
		g_tTXJUartYC_GC[2].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[2].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[2].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[2].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[2].ucDataType						= 0x12;
				
		/*���߹���ң�������3��*/	
		g_tTXJUartYC_GC[2].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[2].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[2].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[2].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[2].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[2].ucW33_PackNo						= 0x02;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[2].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[2].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[2].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[2].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[2].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[2].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[2].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[2].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[2].ucPackEndLow						= 0xD7;
		
		/*���̰�-----------------------------------------------------------------4*/
		g_tTXJUartYC_GC[3].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[3].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[3].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[3].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[3].ucDataType						= 0x12;
				
		/*���߹���ң�������3���*/	
		g_tTXJUartYC_GC[3].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[3].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[3].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[3].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[3].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[3].ucW33_PackNo						= 0x03;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[3].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[3].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[3].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[3].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[3].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[3].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[3].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[3].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[3].ucPackEndLow						= 0xD7;
						
		/*���̰�-----------------------------------------------------------------5*/    
		g_tTXJUartYC_GC[4].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[4].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[4].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[4].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[4].ucDataType						= 0x12;
						
		/*���߹���ң�������5���*/	
		g_tTXJUartYC_GC[4].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[4].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[4].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[4].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[4].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[4].ucW33_PackNo						= 0x04;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[4].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[4].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[4].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[4].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[4].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[4].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[4].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[4].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[4].ucPackEndLow						= 0xD7;
						
		/*���̰�-----------------------------------------------------------------6*/    
		g_tTXJUartYC_GC[5].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[5].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[5].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[5].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[5].ucDataType						= 0x12;
				
		/*���߹���ң�������6���*/	
		g_tTXJUartYC_GC[5].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[5].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[5].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[5].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[5].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[5].ucW33_PackNo						= 0x05;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[5].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[5].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[5].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[5].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[5].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[5].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[5].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[5].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[5].ucPackEndLow						= 0xD7;
						
		/*���̰�-----------------------------------------------------------------7*/     
		g_tTXJUartYC_GC[5].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[5].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[5].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[5].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[5].ucDataType						= 0x12;
				
		/*���߹���ң�������7���*/	
		g_tTXJUartYC_GC[6].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[6].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[6].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[6].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[6].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[6].ucW33_PackNo						= 0x06;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[6].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[6].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[6].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[6].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[6].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[6].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[6].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[6].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[6].ucPackEndLow						= 0xD7;
						
		/*���̰�-----------------------------------------------------------------8*/     
		g_tTXJUartYC_GC[7].ucPackHeadHigh					= 0xEB;
		g_tTXJUartYC_GC[7].ucPackHeadLow					= 0x90;
		g_tTXJUartYC_GC[7].ucDataLenHigh					= 0x00;
		g_tTXJUartYC_GC[7].ucDataLenLow						= 0x80;
		g_tTXJUartYC_GC[7].ucDataType						= 0x12;
				
		/*���߹���ң�������8���*/	
		g_tTXJUartYC_GC[7].aucW0W1_FrameSyn[0]				= 0xEB;
		g_tTXJUartYC_GC[7].aucW0W1_FrameSyn[1]				= 0x90;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[0]			= 0x0B;//0b00001011;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[1]			= 0xB0;//0b10110000;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[2]			= 0;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[3]			= 0;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[4]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[7].aucW2W7_DominantHead[5]			= 0xFF;//0b11111111;
		g_tTXJUartYC_GC[7].ucW8_AuxiliaryHead				= 0x1F;//0b00011111; 
		for(i = 0; i < 24; i++)
		{
			g_tTXJUartYC_GC[7].aucW9W32_AuxiliaryData[i] 	= 0xAA;
		}
		g_tTXJUartYC_GC[7].ucW33_PackNo						= 0x07;
		
		usXorCheckSum = XorCheckSum16((uint16_t *)&g_tTXJUartYC_GC[7].aucW34W125_ValidData[6], 43);
		g_tTXJUartYC_GC[7].aucW126W127_CheckSum[0]			= u16_2_u8(usXorCheckSum, 0);
		g_tTXJUartYC_GC[7].aucW126W127_CheckSum[1]			= u16_2_u8(usXorCheckSum, 1);
						
		g_tTXJUartYC_GC[7].ucCheckSumHigh					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[7].ucDataLenHigh, 131), 1);
		g_tTXJUartYC_GC[7].ucCheckSumLow					= u16_2_u8(AddCheckSum16(&g_tTXJUartYC_GC[7].ucDataLenHigh, 131), 0);
		g_tTXJUartYC_GC[7].ucPackEndHigh					= 0x09;
		g_tTXJUartYC_GC[7].ucPackEndLow						= 0xD7;
	}
}

/************************************************************************
    ��  ��  ��: TXJ_UART_YC_Send
    ����˵��: ����ң�ⷢ��
    �������:
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void TXJ_UART_YC_Send(void)
{
	if(g_ZJState.ucSecondFlag == 1)
	{
		g_ZJState.ucSecondFlag = 0;
		
		if(g_ZJState.ucWX_YCMode == 0)
		{
			/*����������Ҫң������*/
			FPGAUart_SendArray(UART_TXJ_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tTXJUartYC_ZY, sizeof(g_tTXJUartYC_ZY));	
		}
		else
		{
			/*�������߹���ң������*/
			FPGAUart_SendArray(UART_TXJ_TXDATA_ADDR_DEBUG, (uint8_t *)&g_tTXJUartYC_GC, sizeof(g_tTXJUartYC_GC));	
		}
	}

}


/************************************************************************
    ��  ��  ��: GZ_UART_Data_Analysis
    ����˵��: UART�����ֽ����ݣ�У�����
    �������: ��
    �������:
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
void GZ_UART_Data_Analysis(E_UART_OBJECT eUartObject)
{
	uint32_t			uiUartRxDataAddr	= 0;        //���ڽ�������fifo��ַ
	uint32_t			uiUartTxDataAddr	= 0;        //���ڷ�������fifo��ַ
    uint16_t    		usDataIdx       	= 0;        //���ڽ�����������
    uint16_t    		usRxFifoDataNum 	= 0;        //���ڽ������ݸ���
	uint8_t				ucUartByte 			= 0;		//���ڽ��յ��ĵ��ֽ�����
//	uint16_t			usPJPackHead		= 0;        //ƴ�ӵõ��İ�ͷ
//	uint16_t			usPJDataLen			= 0;        //ƴ�ӵõ��İ�˳�����
//	uint8_t				ucDataType			= 0;        //ƴ�ӵõ��İ�˳����Ƶ�bit15 downto bit14������
//	uint16_t			usPJSateSynWord		= 0;        //ƴ�ӵõ��İ�˳����Ƶ�bit13 downto bit0�����
//	uint8_t				ucModeWord			= 0;        //ƴ�ӵõ��İ���
//	uint16_t			usPJCRC				= 0;        //ƴ�ӵõ��Ĺ��ܱ�ʶ��
//	uint16_t			usCalCRC			= 0;        //���ݽ������ݼ���õ���CRC
//	uint16_t			usPJCheckSum		= 0;        //ƴ�ӵõ���У���
//	uint16_t			usCalCheckSum		= 0;		//���ݽ������ݼ���õ���У���
//	uint16_t			usPJPackEnd			= 0;		//ƴ�ӵõ��İ�β
	
	/*��ȡ���ڽ���fifo�е����ݸ���*/
    usRxFifoDataNum = Is_FPGA_Uart_RxFifo_Empty(eUartObject);
	
	switch(eUartObject)
	{
		case eCX://����
			uiUartRxDataAddr = UART_CX_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_CX_TXDATA_ADDR_DEBUG;
			break;
		case eTXJ://ͨ�Ż�
			uiUartRxDataAddr = UART_TXJ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_TXJ_TXDATA_ADDR_DEBUG;
			break;
		case eDC://�ز�
			uiUartRxDataAddr = UART_DC_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_DC_TXDATA_ADDR_DEBUG;
			break;
		case eGZ://����
			uiUartRxDataAddr = UART_GZ_RXDATA_ADDR_DEBUG;
			uiUartTxDataAddr = UART_GZ_TXDATA_ADDR_DEBUG;
			break;
		default:
			break;
	}
	
	if(usRxFifoDataNum)
	{
		for(usDataIdx = 0; usDataIdx < usRxFifoDataNum; usDataIdx++)
        {
			/*��UART�����л�ȡһ���ֽ�*/
			ucUartByte = FPGA_REG_RW_1BYTE(uiUartRxDataAddr);
			
			FPGAUart_SendByte(uiUartTxDataAddr, ucUartByte);
		}
	}
}
