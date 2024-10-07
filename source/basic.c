/*
 * basic.c
 *
 *  Created on: 2023��10��16��
 *      Author: linxiaojun
 */

#include "extern.h"

/*-----------------------------------------------------------
 * ������Uint8ƴ��һ��Uint16
 *----------------------------------------------------------*/
Uint16 u8_2_u16(Uint8 ucData1,Uint8 ucData0)
{
    Uint16 usData = (ucData1<<8) + ucData0;
    return usData;
}

/*-----------------------------------------------------------
* ��һ��Uint16���Uint8
*----------------------------------------------------------*/
Uint8 u16_2_u8(Uint16 usData,Uint8 ucIndex)
{
    Uint8 ucData;
    switch(ucIndex)
    {
        case 0:
            ucData = usData & 0xFF;
            break;
        case 1:
            ucData = (usData & 0xFF00)>>8;
            break;
        default:
            ucData = usData & 0xFF;
            break;
    }
    return ucData;
}

/*-----------------------------------------------------------
* ���ĸ�Uint8ƴ��һ��Uint32
*----------------------------------------------------------*/
Uint32 u8_2_u32(Uint8 ucData3,Uint8 ucData2,Uint8 ucData1,Uint8 ucData0)
{
	Uint32 uiData0 = 0;
	Uint32 uiData1 = 0;
	Uint32 uiData2 = 0;
	Uint32 uiData3 = 0;
	Uint32 uiData = 0;

	uiData0 = ucData0;
	uiData1 = ucData1;
	uiData2 = ucData2;
	uiData3 = ucData3;

    uiData 	= (uiData3<<24) + (uiData2<<16) + (uiData1<<8) + uiData0;

    return uiData;
}

/*-----------------------------------------------------------
* ���ĸ�Uint8ƴ��һ��float
*----------------------------------------------------------*/
float u8_2_flt(Uint8 ucData3,Uint8 ucData2,Uint8 ucData1,Uint8 ucData0)
{
	Uint32 uiData0 = 0;
	Uint32 uiData1 = 0;
	Uint32 uiData2 = 0;
	Uint32 uiData3 = 0;
	Uint32 uiData  = 0;
	float fData = 0;

	uiData0 = ucData0;
	uiData1 = ucData1;
	uiData2 = ucData2;
	uiData3 = ucData3;

    uiData 	= (uiData3<<24) + (uiData2<<16) + (uiData1<<8) + uiData0;
	
	fData = *(float *)&uiData;

    return fData;
}

/*-----------------------------------------------------------
* ��8��Uint8ƴ��һ��double
*----------------------------------------------------------*/
double u8_2_dbl(Uint8 aucData[8])
{
	Uint64 aulData[8];
	Uint64 ulData;
	double dData = 0;
	int i = 0;
	
	for(i = 0; i < 8; i++)
	{
		aulData[i] = aucData[i];
	}

    ulData 	= (aulData[0]<<56) + (aulData[1]<<48) + (aulData[2]<<40) + (aulData[3]<32) + (aulData[4]<<24) + (aulData[5]<<16) + (aulData[6]<<8) + aulData[7];
	
	dData = *(double *)&ulData;

    return dData;
}

/*-----------------------------------------------------------
* ��һ��Uint32���Uint8
*----------------------------------------------------------*/
Uint8 u32_2_u8(Uint32 uiData,Uint8 ucIndex)
{
    Uint8 ucData;
    switch(ucIndex)
    {
        case 0:
            ucData = uiData & 0xFF;
            break;
        case 1:
            ucData = (uiData & 0xFF00)>>8;
            break;
        case 2:
            ucData = (uiData & 0xFF0000)>>16;
            break;
        case 3:
            ucData = (uiData & 0xFF000000)>>24;
            break;
        default:
            ucData = uiData & 0xFF;
            break;
    }
    return ucData;
}

/*-----------------------------------------------------------
* ��һ��float���Uint8
*----------------------------------------------------------*/
Uint8 flt_2_u8(float *fData,Uint8 ucIndex)
{
    Uint8 ucData;

    ucData = *((Uint8 *)fData + ucIndex);

    return ucData;
}


/*ȡֵ*/
Uint32 Get_Bit(Uint32 uData, Uint8 ucBitNum)
{
	return ((Uint32)(uData >> ucBitNum) & 1);
}

/*��λ*/
void Set_Bit(Uint32 *uData, Uint8 ucBitNum)
{
	*uData = ((Uint32)(1 << ucBitNum)) | (*uData);
}

/*����*/
void Clear_Bit(Uint32 *uData, Uint8 ucBitNum)
{
	*uData = ((Uint32)(~(1 << ucBitNum))) & (*uData);
}

/*������������ֵ*/
void Swap_Value(Uint16 *pusdata1, Uint16 *pusdata2)
{
	Uint16 usdata;
	usdata = *pusdata1;
	*(pusdata1) = *(pusdata2);
	*(pusdata2) = usdata;
}

/*sign����*/
Int32 sign(double dinput)
{
	if(dinput < 0.0)
	{
		return -1;
	}
	else if(dinput == 0.0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/*���������Ĵη�*/
double pow_new(double dinput, double dpower)
{
	double dretval;

	if(dinput < 0)
	{
		dinput = -dinput;
		dretval = pow(dinput, dpower);
		dretval = -dretval;
	}
	else
	{
		dretval = pow(dinput, dpower);
	}
	return dretval;
}

/*8λ���У���*/
Uint8 XorCheckSum8(Uint8 *pucStartAddr, Uint32 uiBytNum)
{
	Uint32 	uiIdx = 0;
	Uint8 	ucCheckSum = 0;//У���

	for( uiIdx = 0; uiIdx < uiBytNum; uiIdx++ )
	{
		ucCheckSum = ucCheckSum ^ (*(pucStartAddr + uiIdx));
	}
	return ucCheckSum;
}

/*16λ���У���*/
Uint16 XorCheckSum16(Uint16 *pusStartAddr, Uint32 uiBytNum)
{
	Uint32 	uiIdx = 0;
	Uint16 	usCheckSum = 0;//У���

	for( uiIdx = 0; uiIdx < uiBytNum; uiIdx++ )
	{
		usCheckSum = usCheckSum ^ (*(pusStartAddr + uiIdx));
	}
	return usCheckSum;
}

/*32λ���У���*/
Uint32 XorCheckSum32(Uint32 *puiData, Uint32 uiLength)
{
	Uint32 uiIndex;
	Uint32 uiRetval;

	uiIndex = 0;
	uiRetval = puiData[0];

	for(uiIndex = 1; uiIndex < uiLength; uiIndex++)
	{
		uiRetval= uiRetval ^ puiData[uiIndex];
	}

	return uiRetval;
}

/*8λ�ۼ�У���*/
Uint8 AddCheckSum8(Uint8 *pucStartAddr, Uint32 uiBytNum)
{
	Uint32 	uiIdx = 0;
	Uint8 	ucCheckSum = 0;//У���

	for( uiIdx = 0; uiIdx < uiBytNum; uiIdx++ )
	{
		ucCheckSum = ucCheckSum + (*(pucStartAddr + uiIdx));
	}
	return ucCheckSum;
}

/*16λ�ۼ�У���*/
Uint16 AddCheckSum16(Uint8 *pucStartAddr, Uint32 uiBytNum)
{
	Uint32 	uiIdx = 0;
	Uint16 	usCheckSum = 0;//У���

	for( uiIdx = 0; uiIdx < uiBytNum; uiIdx++ )
	{
		usCheckSum = usCheckSum + pucStartAddr[uiIdx];
	}
	return usCheckSum;
}

/*16λCRCУ��*/
Uint16 CRC16(Uint8 *pucStartAddr, Uint32 uiBytNum)
{
	Uint32 	uiIdx1	= 0;
	Uint32	uiIdx2	= 0;
	Uint16 	usCRC	= 0xFFFF;//У���

	for(uiIdx1 = 0; uiIdx1 < uiBytNum; uiIdx1++)
	{
		usCRC = usCRC ^ (Uint16)pucStartAddr[uiIdx1];

		for(uiIdx2 = 0; uiIdx2 < 8; uiIdx2++)
		{
			if((usCRC & 0x0001))
			{
				usCRC = (usCRC >> 1) & 0xFFFF;
				usCRC = (usCRC ^ 0x1021) & 0xFFFF;
			}
			else
			{
				usCRC = (usCRC >> 1) & 0xFFFF;
			}
		}
	}

	return usCRC;
}

/*16λCRCУ��*/
Uint16 CRC16_2(Uint8 *pucStartAddr, Uint32 uiBytNum)
{
	Uint8	ucData	= 0;
	Uint32	uiIdx2	= 0;
	Uint16	usPoly	= 0x1021;
	Uint16 	usCRCIn	= 0xFFFF;//У���

	while(uiBytNum--)
	{
		ucData = *(pucStartAddr++);
		usCRCIn ^=  (ucData << 8);

		for(uiIdx2 = 0; uiIdx2 < 8; uiIdx2++)
		{
			if(usCRCIn & 0x8000)
			{
				usCRCIn = (usCRCIn << 1) ^ usPoly;
			}
			else
			{
				usCRCIn = usCRCIn << 1;
			}
		}

	}

	return usCRCIn;
}


/************************************************************************
    ��  ��  ��: GetVersion
    ����˵��: ��ȡ�汾��,����-��-����Ϣ
    �������: ��
    �������: ��
    �޸�˵��:
    ��  ��  ��: ������
************************************************************************/
Uint16 GetVersion(void)
{
    char	aucMonthFormat[12][4]	= {"Jan", "Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    char	aucMonth[4] 			= {'\0', };//�·��ַ�
    Uint8	ucIdx					= 0;//����
    Int32	iReturn 				= 0;//�ַ����ȽϷ���ֵ
    Uint8	ucMonth					= 0;//��
    Uint8	ucDay					= 0;//��

    ucDay  = (Uint8)strtoul(((char *)&__DATE__ + 4), NULL, 10);//�� �� ת��Ϊ�޷�������, 10����

    /*ȡ���ַ���ʽ����*/
    memcpy((void *)&aucMonth[0],(void *)&__DATE__, 3);

    for(ucIdx = 0; ucIdx < 12; ucIdx++)
	{
		iReturn = strcmp((const char *)&aucMonthFormat[ucIdx][0], (const char *)&aucMonth[0]);

		if(0 == iReturn)
		{
			ucMonth = ucIdx + 1;//�õ��·ݵ���ֵ
			break;
		}
	}

    return u8_2_u16(ucMonth, ucDay);
}
