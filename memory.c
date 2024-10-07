#include "os.h" 
#include "memory.h"   

/*----------------------------------------------------------*/

void SaveTo3(CPU_INT08U *pDst, CPU_INT08U *pSrc,CPU_INT16U len)
{
    unsigned int i=0;
    for (i=0; i<len; i++)
    {
        pDst[i] = pSrc[i];
		__asm("DSB");
        pDst[GLEN + i] = pSrc[i];
		__asm("DSB");
        pDst[GLEN*2 + i] = pSrc[i];
		__asm("DSB");
    }
}

//???16?3?
void SaveTo3_16(CPU_INT16U *pDst, CPU_INT16U wVal)
{
    CPU_INT08U * pTemp = (CPU_INT08U *)&wVal;           //?wVal????????pTemp
    SaveTo3((CPU_INT08U *)pDst,pTemp,2);                 
}

//???16?3?
void SaveTo3_I16(CPU_INT16S *pDst, CPU_INT16S wVal)
{
    SaveTo3_16((CPU_INT16U *)pDst, (CPU_INT16U)wVal);
}

//???32?3?
void SaveTo3_32(CPU_INT32U *pDst, CPU_INT32U wVal)
{
    CPU_INT08U * pTemp = (CPU_INT08U *)&wVal;
    SaveTo3((CPU_INT08U *)pDst,pTemp,4);

}

//???32?3?
void SaveTo3_I32(CPU_INT32S *pDst, CPU_INT32S wVal)
{
        SaveTo3_32((CPU_INT32U *)pDst, (CPU_INT32U)wVal);
}

//32????3?
void SaveTo3_CPU_FP32(CPU_FP32 *pDst, CPU_FP32 wVal)
{
    CPU_INT08U * pTemp = (CPU_INT08U *)&wVal;
    SaveTo3((CPU_INT08U *)pDst,pTemp,4);
}

//64????3?
void SaveTo3_CPU_FP64(CPU_FP64 *pDst, CPU_FP64 wVal)
{
    CPU_INT08U * pTemp = (CPU_INT08U *)&wVal;
    SaveTo3((CPU_INT08U *)pDst,pTemp,8);
}

/*----------------------------------------------------------*/

//???8???
void getVarIn3_8(CPU_INT08U * pDst)
{
    if (pDst[0] == pDst[GLEN])
    {
        pDst[GLEN*2] = pDst[0];
		__asm("DSB");
    }
    else if (pDst[0] == pDst[GLEN*2])
    {
        pDst[GLEN] = pDst[0];
		__asm("DSB");
    }
    else if (pDst[GLEN] == pDst[GLEN*2])
    {
        pDst[0] = pDst[GLEN];
		__asm("DSB");
    }
    else
    {
        pDst[0] = 0;
		__asm("DSB");
        pDst[GLEN] = 0;
		__asm("DSB");
        pDst[GLEN*2] = 0;
		__asm("DSB");
    }
		
}

//??????16???
CPU_INT16U getVarIn3_16(CPU_INT16U *pDst)
{	
		if (((CPU_INT32U)pDst % 2) == 0)
    {
        if (pDst[0] == pDst[GLEN/2])
        {
            pDst[GLEN] = pDst[0];
			__asm("DSB");
        }
        else if (pDst[0] == pDst[GLEN])
        {
            pDst[GLEN/2] = pDst[0];
			__asm("DSB");
        }
        else if (pDst[GLEN/2] == pDst[GLEN])
        {
            pDst[0] = pDst[GLEN/2];
			__asm("DSB");
        }
        else
        {
            pDst[0] = 0;
			__asm("DSB");
            pDst[GLEN/2] = 0;
			__asm("DSB");
            pDst[GLEN] = 0;
			__asm("DSB");
        }
    }
    else
    {
        getVarIn3_8((CPU_INT08U *)pDst);
        getVarIn3_8((CPU_INT08U *)pDst+1);
    } 
		return MEM_VAL_BIG_TO_HOST_16(pDst[0]);
}

//??????16???
CPU_INT16S getVarIn3_I16(CPU_INT16S * pDst)
{
    return getVarIn3_16((CPU_INT16U *)pDst);
}

//??????32???
CPU_INT32U getVarIn3_32(CPU_INT32U * pDst)
{
    if (((CPU_INT32U)pDst % 4) == 0)
    {
        if (pDst[0] == pDst[GLEN/4])
        {
            pDst[GLEN/2] = pDst[0];
			__asm("DSB");
        }
        else if (pDst[0] == pDst[GLEN/2])
        {
            pDst[GLEN/4] = pDst[0];
			__asm("DSB");
        }
        else if (pDst[GLEN/4] == pDst[GLEN/2])
        {
            pDst[0] = pDst[GLEN/4];
			__asm("DSB");
        }
        else
        {
            pDst[0] = 0;
			__asm("DSB");
            pDst[GLEN/4] = 0;
			__asm("DSB");
            pDst[GLEN/2] = 0;
			__asm("DSB");
        }
    }
    else
    {
        getVarIn3_16((CPU_INT16U *)pDst);
        getVarIn3_16((CPU_INT16U *)pDst+1);
    } 
		return MEM_VAL_BIG_TO_HOST_32(pDst[0]);
}

//??????32???
CPU_INT32S  getVarIn3_I32(CPU_INT32S * pDst)
{
    return getVarIn3_32((CPU_INT32U *)pDst);
}
