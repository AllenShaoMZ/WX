#include "tc.h"
#include "memory.h" 
#include "typedef.h"

extern  GCTM GC;
extern  ZSTM ZS;

CMD_Q         cmdQ[CMDQ_MAXLEN];


void cmdEnqueue(CPU_INT08U * cmdCodeAddr) //指令入队
{
	CPU_INT16U temp_GC_read,temp_GC_write;
	temp_GC_read = getVarIn3_16(&(GC.read));
	temp_GC_write= getVarIn3_16(&(GC.write));
//if((*cpu_state & 0x0007)==0x7 || (*cpu_state & 0x0007)==0x2)  /*有权的计算机在星箭分离后才能发指令*/
	{
		if(((temp_GC_write +1)% CMDQ_MAXLEN)!=temp_GC_read)	 //let is ok?
			{
				Mem_Copy((CPU_INT08U*)&cmdQ[temp_GC_write],(CPU_INT08U*)cmdCodeAddr,3);     //let whether is ok
				temp_GC_write = (temp_GC_write + 1) % CMDQ_MAXLEN; //let store to 3 place
				SaveTo3_16(&(GC.write),MEM_VAL_BIG_TO_HOST_16(temp_GC_write ));
			}
	}
}

