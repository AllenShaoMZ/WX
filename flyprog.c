//#include <interface.h>
#include "typedef.h"
#include "flyprog.h"
#include "memory.h"
#include <stdlib.h>
#include "RS422.h"
#include "yc_init.h"
#include "tc.h"
#include "define.h"
#include "FPGAUart.h"
#include "program_control.h"

extern  GCTM GC;
extern  ZSTM ZS;

extern  GNB_TM   GNB_buf;
extern  XTTC_TM   XTTCA_buf;
extern  XTTC_TM   XTTCB_buf;
extern  PCDU_TM   PCDU_buf;
extern  CPU_INT16U gps_ok_tag;

extern	T_CX_UART_YK_CMD_STATE		g_tCXUartYKCmdState;
extern Mode mode;  // 声明全局变量


//extern  CPU_INT16U yk_mode_tag;

CPU_INT16U *auto_connect_enable_state=(CPU_INT16U *)AUTO_CONNECT_ENABLE_STATE;//0x64300000;
CPU_INT16U *zk_start =(CPU_INT16U *)ZK_START;
CPU_INT16U *auto_switch_enable_state =(CPU_INT16U *)AUTO_SWITCH_ENABLE_STATE;

CPU_INT16U     * const cpu_state = (CPU_INT16U *)CPU_STATE;         /*CPU状态寄存器*/

CPU_INT16U USB_cnt = 0;

// 加热器开关命令表
CMD_Q const temp_cmd_table[HEATER_ROADNUM][2] = {
    {KW1_ADDR, 0xAAAA, KW1_ADDR, 0x5555},  /* 惯组开/关 */
    {KW2_ADDR, 0xAAAA, KW2_ADDR, 0x5555},  /* 星敏开/关 */
    {KW3_ADDR, 0xAAAA, KW3_ADDR, 0x5555},  /* 笼屉开/关 */
    {KW4_ADDR, 0xAAAA, KW4_ADDR, 0x5555},  /* 冷气推力器1开/关 */
    {KW5_ADDR, 0xAAAA, KW5_ADDR, 0x5555},  /* 冷气推力器2开/关 */
    {KW6_ADDR, 0xAAAA, KW6_ADDR, 0x5555},  /* 冷气气瓶开/关 */
    {KW7_ADDR, 0xAAAA, KW7_ADDR, 0x5555},  /* 冷气减压阀开/关 */
    {KW8_ADDR, 0xAAAA, KW8_ADDR, 0x5555},  /* 单组元催化床1开/关 */
    {KW9_ADDR, 0xAAAA, KW9_ADDR, 0x5555},  /* 单组元催化床3开/关 */
    {KW10_ADDR, 0xAAAA, KW10_ADDR, 0x5555}, /* 单组元催化床5开/关 */
    {KW11_ADDR, 0xAAAA, KW11_ADDR, 0x5555}, /* 单组元催化床6开/关 */
    {KW12_ADDR, 0xAAAA, KW12_ADDR, 0x5555}, /* 单组元催化床7开/关 */
    {KW13_ADDR, 0xAAAA, KW13_ADDR, 0x5555}, /* 单组元催化床8开/关 */
    {KW14_ADDR, 0xAAAA, KW14_ADDR, 0x5555}, /* 单组元贮箱开/关 */
    {KW15_ADDR, 0xAAAA, KW15_ADDR, 0x5555}, /* 单组元管路开/关 */
    {KW16_ADDR, 0xAAAA, KW16_ADDR, 0x5555}, /* 单组元压传开/关 */
    {KW17_ADDR, 0xAAAA, KW17_ADDR, 0x5555}, /* 单组元推力器电磁阀1开/关 */
    {KW18_ADDR, 0xAAAA, KW18_ADDR, 0x5555}, /* 单组元推力器电磁阀2开/关 */
    {KW19_ADDR, 0xAAAA, KW19_ADDR, 0x5555}, /* 观瞄组件开/关 */
    {KW20_ADDR, 0xAAAA, KW20_ADDR, 0x5555}, /* ZDB开/关 */
    {KW21_ADDR, 0xAAAA, KW21_ADDR, 0x5555}, /* 蓄电池开/关 */
};

CPU_INT08U  const *FD_on = g_tCXUartYKCmdState.aucK521, 
				  *TJ_on = g_tCXUartYKCmdState.aucK523, 
				  *GM_on = g_tCXUartYKCmdState.aucK525, 
				  *TXJ_on = g_tCXUartYKCmdState.aucK527,  
				  *XM_on = g_tCXUartYKCmdState.aucK529, 
				  *GZ_on = g_tCXUartYKCmdState.aucK5211,   
				  *GNSS_on = g_tCXUartYKCmdState.aucK5213; 
					
CPU_INT08U  const FD_off[4] = {0x00,0x00,0xaa,0x0a}, 
					  TJ_off[4] = {0x00,0x00,0xaa,0x0c},  
					  GM_off[4] = {0x00,0x00,0xaa,0x12}, 
					  TXJ_off[4] = {0x00,0x00,0xaa,0x14},   
					  XM_off[4] = {0x00,0x00,0xaa,0x16}, 
						GZ_off[4] = {0x00,0x00,0xaa,0x16},
					  GNSS_off[4] = {0x00,0x00,0xaa,0x18};

enum {
    KW1,  // 惯组
    KW2,  // 星敏
    KW3,  // 笼屉
    KW4,  // 冷气推力器1
    KW5,  // 冷气推力器2
    KW6,  // 冷气气瓶
    KW7,  // 冷气减压阀
    KW8,  // 单组元催化床1
    KW9,  // 单组元催化床3
    KW10, // 单组元催化床5
    KW11, // 单组元催化床6
    KW12, // 单组元催化床7
    KW13, // 单组元催化床8
    KW14, // 单组元贮箱
    KW15, // 单组元管路
    KW16, // 单组元压传
    KW17, // 单组元推力器电磁阀1
    KW18, // 单组元推力器电磁阀2
    KW19, // 观瞄组件
    KW20, // ZDB
    KW21  // 蓄电池
} HEATER;


CPU_INT16U wk_temperature_table[HEATER_ROADNUM][2];

void urgent_ck (void)
{
		; //待补充
}

void ckstate_update(void)
{
		CPU_INT16U i=0,GC_comp_rule_state=0,GC_prog_state=0;
		CPU_INT16U GC_autowk_permit_tag=0;
			//CPU_INT16U GC_sun_lock_tag=0,GC_ant_lock1_tag=0,GC_ant_lock2_tag=0;
		CPU_INT16U ZS_adjust_mode_tag=0,ZS_G_CLK=0,ZS_G_ALLOWPROG=0,ZS_close_permit_tag=0;
		CPU_INT16U ZS_ck_reset_permit=0,ZS_sc_timing_off_permit=0,ZS_PCDU_reset_permit=0;
		CPU_INT16U ZS_G_ALLOWCHECK=0,GC_Tm_down_tag=0,GC_DelayTm_record_tag=0;
		CPU_INT16U GC_batt_protect_switch=0;//,GC_Tmsc_tag=0,GC_bal_ctrl_allow=0;	
		CPU_INT16U GC_dir_cmd_count=0,GC_syn_ok_cnt=0,GC_vcid_ok_cnt=0,GC_frame_ok_cnt=0,GC_yc_status3=0,GC_energy_management_tag=0;


		CPU_INT16U	* const syn_ok_cnt = (CPU_INT16U	*)SYN_OK_CNT;
		CPU_INT16U	* const vcid_ok_cnt = (CPU_INT16U	*)VCID_OK_CNT;
		CPU_INT16U	* const yc_status3 = (CPU_INT16U	*)YC_STATUS3;
		CPU_INT16U	* const frame_ok_cnt = (CPU_INT16U	*)FRAME_OK_CNT;
		CPU_INT16U	* const USB_dir_cnt  = (CPU_INT16U	*)DIRCMDCNT;
    CPU_INT16U	* const tm_cfg=(CPU_INT16U *) TM_BAUD_SET_REG_ADDR;

    *tm_cfg= getVarIn3_16(&GC.Tm_baud); 
		GC_syn_ok_cnt =(*syn_ok_cnt);
		GC_vcid_ok_cnt =(*vcid_ok_cnt);
		GC_yc_status3 =(*yc_status3);
		GC_frame_ok_cnt =(*frame_ok_cnt);
		GC_dir_cmd_count = (*USB_dir_cnt);

		SaveTo3_16(&GC.syn_ok_cnt,GC_syn_ok_cnt);
		SaveTo3_16(&GC.vcid_ok_cnt,GC_vcid_ok_cnt);
		SaveTo3_16(&GC.yc_status3,GC_yc_status3);
		SaveTo3_16(&GC.frame_ok_cnt,GC_frame_ok_cnt);
		SaveTo3_16(&GC.dir_cmd_count,GC_dir_cmd_count);

		if(USB_cnt != GC_dir_cmd_count)
			{
				SaveTo3_32(&GC.ckzj_timing_count,0);
			}
		USB_cnt = GC_dir_cmd_count;

		GC_energy_management_tag = getVarIn3_16(&GC.energy_management_tag);
		GC_comp_rule_state = getVarIn3_16(&GC.comp_rule_state);
		GC_prog_state = getVarIn3_16(&GC.prog_state);

		GC_autowk_permit_tag = getVarIn3_16(&GC.autowk_permit_tag);
		ZS_adjust_mode_tag = getVarIn3_16(&ZS.adjust_mode_tag);
		ZS_G_CLK = getVarIn3_16(&ZS.G_CLK);
		ZS_G_ALLOWPROG = getVarIn3_16(&ZS.G_ALLOWPROG);
		ZS_close_permit_tag = getVarIn3_16(&ZS.close_permit_tag);
		ZS_PCDU_reset_permit = getVarIn3_16(&ZS.PCDU_reset_permit);
		ZS_ck_reset_permit= getVarIn3_16(&ZS.ck_reset_permit);
		ZS_sc_timing_off_permit = getVarIn3_16(&ZS.sc_timing_off_permit);

		ZS_G_ALLOWCHECK = getVarIn3_16(&ZS.G_ALLOWCHECK);
		GC_Tm_down_tag = getVarIn3_16(&GC.Tm_down_tag);

		GC_batt_protect_switch = getVarIn3_16(&GC.batt_protect_switch);


		for(i=0; i<3; i++)
			{
				if(DEF_BIT_IS_SET((*cpu_state),DEF_BIT16(i))) //CPU状态寄存器？？？？
					{
						DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(i));
					}
				else
					{
						DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(i));
					}
			}

		if(GC_Tm_down_tag ==0x5555)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(8));
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(8));
			}

		if(GC_DelayTm_record_tag ==0x5555)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(9));
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(9));
			}
		if(*auto_switch_enable_state ==0x5555)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(10));
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(10));
			}
		if(*auto_connect_enable_state ==0x5555)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(11));
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(11));
			}

		if(GC_batt_protect_switch == 0x5555)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(12));
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(12));
			}

		if(0xAAAA==getVarIn3_16(&GC.first_power_tag))
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(14));//卫星首次上电标志
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(14));
			}
			
		if(0xAAAA==*zk_start)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(15));//姿控软件运行
			}
		else 
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(15));//姿控软件未运行
			}

		if(GC_autowk_permit_tag ==0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(0));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(0));
			}			
		if(GC_energy_management_tag ==0x5555)
				{
					DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(1));
				}
			else
				{
					DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(1));
				}

		if (ZS_adjust_mode_tag == 0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(3));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(3));
			}

		if (ZS_G_CLK == 0x146f)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(4));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(4));
			}

		if (ZS_G_ALLOWPROG == 0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(5));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(5));
			}

		if (gps_ok_tag== 0x6666)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(6));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(6));
			}

		if (ZS_close_permit_tag== 0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(7));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(7));
			}

		if ((ZS_ck_reset_permit & 0x000f) == 0x0005)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(8));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(8));
			}
		

		if ((ZS_ck_reset_permit & 0x00f0)== 0x0050)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(9));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(9));
			}
		

		if ((ZS_sc_timing_off_permit & 0x000f) == 0x0005)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(10));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(10));
			}
		if ((ZS_sc_timing_off_permit & 0x00f0) == 0x0050)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(11));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(11));
			}
		if (ZS_PCDU_reset_permit == 0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(12));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(12));
			}
		if (ZS_G_ALLOWCHECK == 0x5555)
			{
				DEF_BIT_SET_16(GC_prog_state,DEF_BIT16(13));
			}
		else
			{
				DEF_BIT_CLR_16(GC_prog_state,DEF_BIT16(13));
			}


		SaveTo3_16(&GC.comp_rule_state,GC_comp_rule_state);
		SaveTo3_16(&GC.prog_state,GC_prog_state);
}



void temperature_control(void)
{
	CPU_INT16U temp_val  = 0,wkLimit_Cur  = 0;
	CPU_INT08U temp_wk_point  = 0;
	CPU_INT08U wk_handle_point  =0;
	CPU_INT16U ZS_wk_select_tag=0,ZS_wk_permit_tag=0,GC_autowk_permit_tag=0;
	CPU_INT16U GC_Zkfault_temp=0;
	
	CPU_INT16U *current_wkLimit_ptr_tbl;
	
	//正常工作模式的门限表
	CPU_INT16U *wkLimit_ptr_tbl[HEATER_ROADNUM] = {
    (CPU_INT16U *)&(ZS.inertialUnit_heat_limit),   // 惯组加热器门限
    (CPU_INT16U *)&(ZS.starSensor_heat_limit),     // 星敏加热器门限
    (CPU_INT16U *)&(ZS.cage_heat_limit),           // 笼屉加热器门限
    (CPU_INT16U *)&(ZS.coolThrust1_limit),         // 冷气推力器1加热器门限
    (CPU_INT16U *)&(ZS.coolThrust2_limit),         // 冷气推力器2加热器门限
    (CPU_INT16U *)&(ZS.coolBottle_limit),          // 冷气气瓶加热器门限
    (CPU_INT16U *)&(ZS.coolValve_limit),           // 冷气减压阀加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed1_limit),       // 单组元催化床1加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed3_limit),       // 单组元催化床3加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed5_limit),       // 单组元催化床5加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed6_limit),       // 单组元催化床6加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed7_limit),       // 单组元催化床7加热器门限
    (CPU_INT16U *)&(ZS.catalyticBed8_limit),       // 单组元催化床8加热器门限
    (CPU_INT16U *)&(ZS.fuelTank_limit),            // 单组元贮箱加热器门限
    (CPU_INT16U *)&(ZS.fuelLine_limit),            // 单组元管路加热器门限
    (CPU_INT16U *)&(ZS.pressureSensor_limit),      // 压力传感器加热器门限
    (CPU_INT16U *)&(ZS.solenoidValve1_limit),      // 电磁阀1加热器门限
    (CPU_INT16U *)&(ZS.solenoidValve2_limit),      // 电磁阀2加热器门限
    (CPU_INT16U *)&(ZS.observationUnit_limit),     // 观瞄组件加热器门限
    (CPU_INT16U *)&(ZS.ZDB_limit),                 // ZDB加热器门限
    (CPU_INT16U *)&(ZS.batteryTemp_limit)          // 蓄电池温度状态加热器门限
};

	//存储热控模式下的门限表
	CPU_INT16U *storage_wkLimit_ptr_tbl[HEATER_ROADNUM] = {
    (CPU_INT16U *)&(ZS.inertialUnit_storage_limit),   // 惯组加热器存储模式门限
    (CPU_INT16U *)&(ZS.starSensor_storage_limit),     // 星敏加热器存储模式门限
    (CPU_INT16U *)&(ZS.cage_storage_limit),           // 笼屉加热器存储模式门限
    (CPU_INT16U *)&(ZS.coolThrust1_storage_limit),    // 冷气推力器1存储模式门限
    (CPU_INT16U *)&(ZS.coolThrust2_storage_limit),    // 冷气推力器2存储模式门限
    (CPU_INT16U *)&(ZS.coolBottle_storage_limit),     // 冷气气瓶存储模式门限
    (CPU_INT16U *)&(ZS.coolValve_storage_limit),      // 冷气减压阀存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed1_storage_limit),  // 单组元催化床1存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed3_storage_limit),  // 单组元催化床3存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed5_storage_limit),  // 单组元催化床5存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed6_storage_limit),  // 单组元催化床6存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed7_storage_limit),  // 单组元催化床7存储模式门限
    (CPU_INT16U *)&(ZS.catalyticBed8_storage_limit),  // 单组元催化床8存储模式门限
    (CPU_INT16U *)&(ZS.fuelTank_storage_limit),       // 单组元贮箱存储模式门限
    (CPU_INT16U *)&(ZS.fuelLine_storage_limit),       // 单组元管路存储模式门限
    (CPU_INT16U *)&(ZS.pressureSensor_storage_limit), // 压力传感器存储模式门限
    (CPU_INT16U *)&(ZS.solenoidValve1_storage_limit), // 电磁阀1存储模式门限
    (CPU_INT16U *)&(ZS.solenoidValve2_storage_limit), // 电磁阀2存储模式门限
    (CPU_INT16U *)&(ZS.observationUnit_storage_limit),// 观瞄组件存储模式门限
    (CPU_INT16U *)&(ZS.ZDB_storage_limit),            // ZDB存储模式门限
    (CPU_INT16U *)&(ZS.batteryTemp_storage_limit)     // 蓄电池温度状态存储模式门限
};
	
	if (mode == STORAGE_MODE) {
			current_wkLimit_ptr_tbl = *storage_wkLimit_ptr_tbl;  // 使用存储模式门限表
	} else {
			current_wkLimit_ptr_tbl = *wkLimit_ptr_tbl;  // 使用正常工作模式门限表
	}

	ZS_wk_permit_tag = getVarIn3_16(&ZS.wk_permit_tag);
	GC_autowk_permit_tag = getVarIn3_16(&GC.autowk_permit_tag);
	ZS_wk_select_tag = getVarIn3_16(&ZS.wk_select_tag);
	GC_Zkfault_temp = getVarIn3_16(&GC.Zkfault);

	if(GC_autowk_permit_tag == 0x5555)
		{
			for (wk_handle_point=0; wk_handle_point<HEATER_ROADNUM; wk_handle_point++)
				{
					temp_val = 0x0001 << wk_handle_point;
					if((ZS_wk_permit_tag & temp_val) != 0)
						{
							if((ZS_wk_select_tag & temp_val) != 0)
								{
									temp_wk_point = 1;
								}
							else
								{
									temp_wk_point = 0;
								}
							wkLimit_Cur = getVarIn3_16(&current_wkLimit_ptr_tbl[wk_handle_point]);
							if((GC_Zkfault_temp& 0x0ff) == 0x0ff )
								{
									wkLimit_Cur = wkLimit_Cur + ((ZS.urgent_wk_limit & 0x00ff) << 8) + (ZS.urgent_wk_limit & 0x00ff);
								}
							if((wk_temperature_table[wk_handle_point][temp_wk_point] <=(wkLimit_Cur & 0x00ff)) && (DEF_BIT_IS_CLR(GC.heater_state,DEF_BIT16(wk_handle_point))))
								{
									cmdEnqueue((CPU_INT08U*)&temp_cmd_table[wk_handle_point][0]);//开加热器指令
								}
							else if((wk_temperature_table[wk_handle_point][temp_wk_point] >= ((wkLimit_Cur >> 8) & 0x00ff)) && (DEF_BIT_IS_SET(GC.heater_state,DEF_BIT16(wk_handle_point))))
								{
									cmdEnqueue((CPU_INT08U*)&temp_cmd_table[wk_handle_point][1]);//关加热器指令
								}
						}
				}
		}
}

void get_wk_data (void)
{
	CPU_INT16U GC_heater_state=0;
    // 获取21个加热器的温度数据
    wk_temperature_table[KW1][0] = GNB_buf.groupInertiaTemp >> 8;   // 惯组
    wk_temperature_table[KW2][0] = GNB_buf.starSensorTemp >> 8;     // 星敏
    wk_temperature_table[KW3][0] = GNB_buf.cageTemp >> 8;           // 笼屉
    wk_temperature_table[KW4][0] = GNB_buf.coolThrust1Temp >> 8;    // 冷气推力器1
    wk_temperature_table[KW5][0] = GNB_buf.coolThrust2Temp >> 8;    // 冷气推力器2
    wk_temperature_table[KW6][0] = GNB_buf.coolBottleTemp >> 8;     // 冷气气瓶
    wk_temperature_table[KW7][0] = GNB_buf.coolValveTemp >> 8;      // 冷气减压阀
    wk_temperature_table[KW8][0] = GNB_buf.catalyticBed1Temp >> 8;  // 单组元催化床1
    wk_temperature_table[KW9][0] = GNB_buf.catalyticBed3Temp>> 8;  // 单组元催化床3
    wk_temperature_table[KW10][0] = GNB_buf.catalyticBed5Temp >> 8; // 单组元催化床5
    wk_temperature_table[KW11][0] = GNB_buf.catalyticBed6Temp >> 8; // 单组元催化床6
    wk_temperature_table[KW12][0] = GNB_buf.catalyticBed7Temp >> 8; // 单组元催化床7
    wk_temperature_table[KW13][0] = GNB_buf.catalyticBed8Temp >> 8; // 单组元催化床8
    wk_temperature_table[KW14][0] = GNB_buf.fuelTankTemp >> 8;      // 单组元贮箱
    wk_temperature_table[KW15][0] = GNB_buf.fuelLineTemp >> 8;      // 单组元管路
    wk_temperature_table[KW16][0] = GNB_buf.pressureSensorTemp >> 8; // 单组元压传
    wk_temperature_table[KW17][0] = GNB_buf.solenoidValve1Temp >> 8; // 单组元推力器电磁阀1
    wk_temperature_table[KW18][0] = GNB_buf.solenoidValve2Temp >> 8; // 单组元推力器电磁阀2
    wk_temperature_table[KW19][0] = GNB_buf.observationTemp >> 8;    // 观瞄组件
		wk_temperature_table[KW20][0] = GNB_buf.ZDBTemp >> 8; // zdb
		wk_temperature_table[KW21][0] = GNB_buf.batteryTemp >> 8; // 蓄电池

	
	GC_heater_state=GNB_buf.satSeparated_1;
	SaveTo3_16(&GC.heater_state,GC_heater_state);
}


