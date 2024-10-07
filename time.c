/*邵明振修改 */

#include "time.h"
#include <cpu.h>
#include <math.h>
#include "typedef.h"
#include "os.h"
#include  <lib_mem.h>
#include "memory.h"

extern   ZSTM  ZS;
extern   GCTM  GC;

CPU_INT16U   * const Time_Reg_Addr = (CPU_INT16U * )TIME_REG_ADD;//时间寄存器高16位

OS_TICK last_tick=0;

T_TIME     TCPGS= {0,0}; //接收GPS中断时刻的系统钟值
CPU_INT16U gps_ok_tag = 0;	 //GPS可用标志
CPU_INT16U gps_ok_count = 0;		//GPS正确计数
CPU_INT16U gps_error_count = 0; //GPS错误计数

//entry_manage：秒入口维护程序
void entry_manage(void)
{
	CPU_INT16U enter_200ms_tmp;
	CPU_INT16U enter_400ms_tmp;
	CPU_INT16U enter_1s_tmp;
	CPU_INT16U enter_2s_tmp;
	CPU_INT16U enter_5s_tmp;
	CPU_INT16U enter_1m_tmp;

	enter_200ms_tmp = getVarIn3_16(&GC.enter_200ms);
	enter_200ms_tmp = (enter_200ms_tmp & 0x1) + 1 ;
	if (enter_200ms_tmp >= 2)
		{
			enter_200ms_tmp = 0;
		}
	SaveTo3_16(&GC.enter_200ms,MEM_VAL_BIG_TO_HOST_16(enter_200ms_tmp));

	enter_400ms_tmp = getVarIn3_16(&GC.enter_400ms);
	enter_400ms_tmp = (enter_400ms_tmp & 0x3) + 1 ;
	if (enter_400ms_tmp >= 4)
		{
			enter_400ms_tmp = 0;
		}
	SaveTo3_16(&GC.enter_400ms,MEM_VAL_BIG_TO_HOST_16(enter_400ms_tmp));

	enter_1s_tmp = getVarIn3_16(&GC.enter_1s);
	enter_1s_tmp = (enter_1s_tmp & 0x7) + 1 ;
	if (enter_1s_tmp >= 10)
		{
			enter_1s_tmp = 0;
		}
	SaveTo3_16(&GC.enter_1s,MEM_VAL_BIG_TO_HOST_16(enter_1s_tmp));

	enter_2s_tmp = getVarIn3_16(&GC.enter_2s);
	enter_2s_tmp = (enter_2s_tmp & 0xf) + 1;
	if (enter_2s_tmp >= 20)
		{
			enter_2s_tmp = 0;
		}
	SaveTo3_16(&GC.enter_2s,MEM_VAL_BIG_TO_HOST_16(enter_2s_tmp));

	enter_5s_tmp = getVarIn3_16(&GC.enter_5s);
	enter_5s_tmp = (enter_5s_tmp & 0x3f) + 1;
	if (enter_5s_tmp >= 50)
		{
			enter_5s_tmp = 0;
		}
	SaveTo3_16(&GC.enter_5s,MEM_VAL_BIG_TO_HOST_16(enter_5s_tmp));

	enter_1m_tmp = getVarIn3_16(&GC.enter_1m);
	enter_1m_tmp = (enter_1m_tmp & 0x1ff) + 1 ;
	if (enter_1m_tmp >= 600)
		{
			enter_1m_tmp = 0;
		}
	SaveTo3_16(&GC.enter_1m,MEM_VAL_BIG_TO_HOST_16(enter_1m_tmp)); //let src and dst is the same ,is it correct?

}

CPU_INT64U  get_time_48bit()
{
		CPU_INT64U time=0;

   time =(*(CPU_INT64U *)Time_Reg_Addr)>>16;

		return MEM_VAL_BIG_TO_HOST_32(time);

}

void systime_add(void)
{

	CPU_INT16U temp_GC_SoftTime_mS;
	CPU_INT32U temp_GC_SoftTime_S;
	OS_TICK delt_time=0,current_tick=0;
	CPU_SR_ALLOC();

	temp_GC_SoftTime_S = getVarIn3_32(&(GC.SoftTime_S));
	temp_GC_SoftTime_mS = getVarIn3_16(&(GC.SoftTime_mS));

	if(ZS.G_CLK == 0x146f)
	 {
			// current_tick =	*hard_clk /1000;
                        
			current_tick =	get_time_48bit() & 0xffffffff;

	 }
	else
		{
			//current_tick =	CPU_TS_Get32() /1000;
			current_tick =	CPU_TS_Get32()/100;
		}

	delt_time = (current_tick - last_tick + 5)/10;

	if((delt_time > 120) && (delt_time < 130))
		{
			temp_GC_SoftTime_mS = temp_GC_SoftTime_mS + delt_time ;
		}
	else
		{
			temp_GC_SoftTime_mS = temp_GC_SoftTime_mS +125;
		}
	if(temp_GC_SoftTime_mS >=SECLENGTH_INMS)
		{
			temp_GC_SoftTime_S = temp_GC_SoftTime_S + 1;
			temp_GC_SoftTime_mS = temp_GC_SoftTime_mS - 1000;
		}
	//CSP_Dbg_Printf("SoftTime_S =%d\n",GC->SoftTime_S);
	//CSP_Dbg_Printf("SoftTime_mS=%d\n",GC->SoftTime_mS);
	CPU_CRITICAL_ENTER();/* 关中断*/
		
		/* shao 保存*/ 
	SaveTo3_32(&GC.SoftTime_S,MEM_VAL_BIG_TO_HOST_32(temp_GC_SoftTime_S));
	SaveTo3_16(&GC.SoftTime_mS,MEM_VAL_BIG_TO_HOST_16(temp_GC_SoftTime_mS));

	last_tick = current_tick;
//  CSP_Dbg_Printf("current_tick =%d\n",current_tick);
	CPU_CRITICAL_EXIT();/* 开中断*/

}

//gps_ok_proce：gps数据可用标志处理
void gps_ok_proce (void)
{
	CPU_INT16U enter_5s_tmp;

	enter_5s_tmp = getVarIn3_16(&GC.enter_5s);
	if (gps_ok_tag == 0x6666)
		{
			gps_error_count = (gps_error_count & 0x1f) + 1;
			if (gps_error_count >= 20)
				{
					gps_ok_tag = 0;
					SaveTo3_16(&GC.enter_5s,0);
					gps_ok_count = 0;
				}
		}
	else
		{
			if (enter_5s_tmp >= 39 )
				{
					if (gps_ok_count < 3)
						{
							gps_ok_count = 0;
						}
					else
						{
							gps_ok_tag = 0x6666;
						}
				}
		}
}
