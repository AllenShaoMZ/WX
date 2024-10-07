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
extern Mode mode;  // ����ȫ�ֱ���


//extern  CPU_INT16U yk_mode_tag;

CPU_INT16U *auto_connect_enable_state=(CPU_INT16U *)AUTO_CONNECT_ENABLE_STATE;//0x64300000;
CPU_INT16U *zk_start =(CPU_INT16U *)ZK_START;
CPU_INT16U *auto_switch_enable_state =(CPU_INT16U *)AUTO_SWITCH_ENABLE_STATE;

CPU_INT16U     * const cpu_state = (CPU_INT16U *)CPU_STATE;         /*CPU״̬�Ĵ���*/

CPU_INT16U USB_cnt = 0;

// ���������������
CMD_Q const temp_cmd_table[HEATER_ROADNUM][2] = {
    {KW1_ADDR, 0xAAAA, KW1_ADDR, 0x5555},  /* ���鿪/�� */
    {KW2_ADDR, 0xAAAA, KW2_ADDR, 0x5555},  /* ������/�� */
    {KW3_ADDR, 0xAAAA, KW3_ADDR, 0x5555},  /* ���뿪/�� */
    {KW4_ADDR, 0xAAAA, KW4_ADDR, 0x5555},  /* ����������1��/�� */
    {KW5_ADDR, 0xAAAA, KW5_ADDR, 0x5555},  /* ����������2��/�� */
    {KW6_ADDR, 0xAAAA, KW6_ADDR, 0x5555},  /* ������ƿ��/�� */
    {KW7_ADDR, 0xAAAA, KW7_ADDR, 0x5555},  /* ������ѹ����/�� */
    {KW8_ADDR, 0xAAAA, KW8_ADDR, 0x5555},  /* ����Ԫ�߻���1��/�� */
    {KW9_ADDR, 0xAAAA, KW9_ADDR, 0x5555},  /* ����Ԫ�߻���3��/�� */
    {KW10_ADDR, 0xAAAA, KW10_ADDR, 0x5555}, /* ����Ԫ�߻���5��/�� */
    {KW11_ADDR, 0xAAAA, KW11_ADDR, 0x5555}, /* ����Ԫ�߻���6��/�� */
    {KW12_ADDR, 0xAAAA, KW12_ADDR, 0x5555}, /* ����Ԫ�߻���7��/�� */
    {KW13_ADDR, 0xAAAA, KW13_ADDR, 0x5555}, /* ����Ԫ�߻���8��/�� */
    {KW14_ADDR, 0xAAAA, KW14_ADDR, 0x5555}, /* ����Ԫ���俪/�� */
    {KW15_ADDR, 0xAAAA, KW15_ADDR, 0x5555}, /* ����Ԫ��·��/�� */
    {KW16_ADDR, 0xAAAA, KW16_ADDR, 0x5555}, /* ����Ԫѹ����/�� */
    {KW17_ADDR, 0xAAAA, KW17_ADDR, 0x5555}, /* ����Ԫ��������ŷ�1��/�� */
    {KW18_ADDR, 0xAAAA, KW18_ADDR, 0x5555}, /* ����Ԫ��������ŷ�2��/�� */
    {KW19_ADDR, 0xAAAA, KW19_ADDR, 0x5555}, /* ���������/�� */
    {KW20_ADDR, 0xAAAA, KW20_ADDR, 0x5555}, /* ZDB��/�� */
    {KW21_ADDR, 0xAAAA, KW21_ADDR, 0x5555}, /* ���ؿ�/�� */
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
    KW1,  // ����
    KW2,  // ����
    KW3,  // ����
    KW4,  // ����������1
    KW5,  // ����������2
    KW6,  // ������ƿ
    KW7,  // ������ѹ��
    KW8,  // ����Ԫ�߻���1
    KW9,  // ����Ԫ�߻���3
    KW10, // ����Ԫ�߻���5
    KW11, // ����Ԫ�߻���6
    KW12, // ����Ԫ�߻���7
    KW13, // ����Ԫ�߻���8
    KW14, // ����Ԫ����
    KW15, // ����Ԫ��·
    KW16, // ����Ԫѹ��
    KW17, // ����Ԫ��������ŷ�1
    KW18, // ����Ԫ��������ŷ�2
    KW19, // �������
    KW20, // ZDB
    KW21  // ����
} HEATER;


CPU_INT16U wk_temperature_table[HEATER_ROADNUM][2];

void urgent_ck (void)
{
		; //������
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
				if(DEF_BIT_IS_SET((*cpu_state),DEF_BIT16(i))) //CPU״̬�Ĵ�����������
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
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(14));//�����״��ϵ��־
			}
		else
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(14));
			}
			
		if(0xAAAA==*zk_start)
			{
				DEF_BIT_SET_16(GC_comp_rule_state,DEF_BIT16(15));//�˿��������
			}
		else 
			{
				DEF_BIT_CLR_16(GC_comp_rule_state,DEF_BIT16(15));//�˿����δ����
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
	
	//��������ģʽ�����ޱ�
	CPU_INT16U *wkLimit_ptr_tbl[HEATER_ROADNUM] = {
    (CPU_INT16U *)&(ZS.inertialUnit_heat_limit),   // �������������
    (CPU_INT16U *)&(ZS.starSensor_heat_limit),     // ��������������
    (CPU_INT16U *)&(ZS.cage_heat_limit),           // �������������
    (CPU_INT16U *)&(ZS.coolThrust1_limit),         // ����������1����������
    (CPU_INT16U *)&(ZS.coolThrust2_limit),         // ����������2����������
    (CPU_INT16U *)&(ZS.coolBottle_limit),          // ������ƿ����������
    (CPU_INT16U *)&(ZS.coolValve_limit),           // ������ѹ������������
    (CPU_INT16U *)&(ZS.catalyticBed1_limit),       // ����Ԫ�߻���1����������
    (CPU_INT16U *)&(ZS.catalyticBed3_limit),       // ����Ԫ�߻���3����������
    (CPU_INT16U *)&(ZS.catalyticBed5_limit),       // ����Ԫ�߻���5����������
    (CPU_INT16U *)&(ZS.catalyticBed6_limit),       // ����Ԫ�߻���6����������
    (CPU_INT16U *)&(ZS.catalyticBed7_limit),       // ����Ԫ�߻���7����������
    (CPU_INT16U *)&(ZS.catalyticBed8_limit),       // ����Ԫ�߻���8����������
    (CPU_INT16U *)&(ZS.fuelTank_limit),            // ����Ԫ�������������
    (CPU_INT16U *)&(ZS.fuelLine_limit),            // ����Ԫ��·����������
    (CPU_INT16U *)&(ZS.pressureSensor_limit),      // ѹ������������������
    (CPU_INT16U *)&(ZS.solenoidValve1_limit),      // ��ŷ�1����������
    (CPU_INT16U *)&(ZS.solenoidValve2_limit),      // ��ŷ�2����������
    (CPU_INT16U *)&(ZS.observationUnit_limit),     // �����������������
    (CPU_INT16U *)&(ZS.ZDB_limit),                 // ZDB����������
    (CPU_INT16U *)&(ZS.batteryTemp_limit)          // �����¶�״̬����������
};

	//�洢�ȿ�ģʽ�µ����ޱ�
	CPU_INT16U *storage_wkLimit_ptr_tbl[HEATER_ROADNUM] = {
    (CPU_INT16U *)&(ZS.inertialUnit_storage_limit),   // ����������洢ģʽ����
    (CPU_INT16U *)&(ZS.starSensor_storage_limit),     // �����������洢ģʽ����
    (CPU_INT16U *)&(ZS.cage_storage_limit),           // ����������洢ģʽ����
    (CPU_INT16U *)&(ZS.coolThrust1_storage_limit),    // ����������1�洢ģʽ����
    (CPU_INT16U *)&(ZS.coolThrust2_storage_limit),    // ����������2�洢ģʽ����
    (CPU_INT16U *)&(ZS.coolBottle_storage_limit),     // ������ƿ�洢ģʽ����
    (CPU_INT16U *)&(ZS.coolValve_storage_limit),      // ������ѹ���洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed1_storage_limit),  // ����Ԫ�߻���1�洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed3_storage_limit),  // ����Ԫ�߻���3�洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed5_storage_limit),  // ����Ԫ�߻���5�洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed6_storage_limit),  // ����Ԫ�߻���6�洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed7_storage_limit),  // ����Ԫ�߻���7�洢ģʽ����
    (CPU_INT16U *)&(ZS.catalyticBed8_storage_limit),  // ����Ԫ�߻���8�洢ģʽ����
    (CPU_INT16U *)&(ZS.fuelTank_storage_limit),       // ����Ԫ����洢ģʽ����
    (CPU_INT16U *)&(ZS.fuelLine_storage_limit),       // ����Ԫ��·�洢ģʽ����
    (CPU_INT16U *)&(ZS.pressureSensor_storage_limit), // ѹ���������洢ģʽ����
    (CPU_INT16U *)&(ZS.solenoidValve1_storage_limit), // ��ŷ�1�洢ģʽ����
    (CPU_INT16U *)&(ZS.solenoidValve2_storage_limit), // ��ŷ�2�洢ģʽ����
    (CPU_INT16U *)&(ZS.observationUnit_storage_limit),// ��������洢ģʽ����
    (CPU_INT16U *)&(ZS.ZDB_storage_limit),            // ZDB�洢ģʽ����
    (CPU_INT16U *)&(ZS.batteryTemp_storage_limit)     // �����¶�״̬�洢ģʽ����
};
	
	if (mode == STORAGE_MODE) {
			current_wkLimit_ptr_tbl = *storage_wkLimit_ptr_tbl;  // ʹ�ô洢ģʽ���ޱ�
	} else {
			current_wkLimit_ptr_tbl = *wkLimit_ptr_tbl;  // ʹ����������ģʽ���ޱ�
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
									cmdEnqueue((CPU_INT08U*)&temp_cmd_table[wk_handle_point][0]);//��������ָ��
								}
							else if((wk_temperature_table[wk_handle_point][temp_wk_point] >= ((wkLimit_Cur >> 8) & 0x00ff)) && (DEF_BIT_IS_SET(GC.heater_state,DEF_BIT16(wk_handle_point))))
								{
									cmdEnqueue((CPU_INT08U*)&temp_cmd_table[wk_handle_point][1]);//�ؼ�����ָ��
								}
						}
				}
		}
}

void get_wk_data (void)
{
	CPU_INT16U GC_heater_state=0;
    // ��ȡ21�����������¶�����
    wk_temperature_table[KW1][0] = GNB_buf.groupInertiaTemp >> 8;   // ����
    wk_temperature_table[KW2][0] = GNB_buf.starSensorTemp >> 8;     // ����
    wk_temperature_table[KW3][0] = GNB_buf.cageTemp >> 8;           // ����
    wk_temperature_table[KW4][0] = GNB_buf.coolThrust1Temp >> 8;    // ����������1
    wk_temperature_table[KW5][0] = GNB_buf.coolThrust2Temp >> 8;    // ����������2
    wk_temperature_table[KW6][0] = GNB_buf.coolBottleTemp >> 8;     // ������ƿ
    wk_temperature_table[KW7][0] = GNB_buf.coolValveTemp >> 8;      // ������ѹ��
    wk_temperature_table[KW8][0] = GNB_buf.catalyticBed1Temp >> 8;  // ����Ԫ�߻���1
    wk_temperature_table[KW9][0] = GNB_buf.catalyticBed3Temp>> 8;  // ����Ԫ�߻���3
    wk_temperature_table[KW10][0] = GNB_buf.catalyticBed5Temp >> 8; // ����Ԫ�߻���5
    wk_temperature_table[KW11][0] = GNB_buf.catalyticBed6Temp >> 8; // ����Ԫ�߻���6
    wk_temperature_table[KW12][0] = GNB_buf.catalyticBed7Temp >> 8; // ����Ԫ�߻���7
    wk_temperature_table[KW13][0] = GNB_buf.catalyticBed8Temp >> 8; // ����Ԫ�߻���8
    wk_temperature_table[KW14][0] = GNB_buf.fuelTankTemp >> 8;      // ����Ԫ����
    wk_temperature_table[KW15][0] = GNB_buf.fuelLineTemp >> 8;      // ����Ԫ��·
    wk_temperature_table[KW16][0] = GNB_buf.pressureSensorTemp >> 8; // ����Ԫѹ��
    wk_temperature_table[KW17][0] = GNB_buf.solenoidValve1Temp >> 8; // ����Ԫ��������ŷ�1
    wk_temperature_table[KW18][0] = GNB_buf.solenoidValve2Temp >> 8; // ����Ԫ��������ŷ�2
    wk_temperature_table[KW19][0] = GNB_buf.observationTemp >> 8;    // �������
		wk_temperature_table[KW20][0] = GNB_buf.ZDBTemp >> 8; // zdb
		wk_temperature_table[KW21][0] = GNB_buf.batteryTemp >> 8; // ����

	
	GC_heater_state=GNB_buf.satSeparated_1;
	SaveTo3_16(&GC.heater_state,GC_heater_state);
}


