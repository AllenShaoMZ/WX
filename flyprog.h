#ifndef _FLYPROG_H_
#define _FLYPROG_H_

#include <math.h>

#include "os.h"  


//#define CKZJDIRCMDCNT  0x60400002         //����м�
#define DIRCMDCNT     0x60200002       //��ƵӦ���ֱ��ָ������Ĵ���
#define SYN_OK_CNT    0x60200006       //ͬ��ͷ���ϼ���
#define VCID_OK_CNT   0x60200008       //VCID���ϼ���
#define YC_STATUS3   0x6020000A       //ң��״̬ң��3
//Bit3~bit0�������ŵ�ʶ����Ϣ
//bit4��������Դ��0�����棬1���ӽ���
//bit5��BCH����״̬��0����ȷ��1������
//bit6��CRCУ��״̬��0����ȷ��1������
//bit7��ң��ת��״̬��1��������0����ת��
#define FRAME_OK_CNT  0x6020000E       //֡��ʽ���ϼ���
#define SECOf1WEEK   0x93A80 			//1�ܵ�����ֵ
#define SECOf25MIN   0x05DC 			//25min������ֵ

#define HEATER_ROADNUM 21               //������·��


#define AUTO_CONNECT_ENABLE_STATE 0x64300000 //������ͨʹ�ܱ�־
#define ZK_START  0x64300010           //�˿����������־
#define FIRST_POWER  0x64300020       //�����״��ϵ��־
#define AUTO_SWITCH_ENABLE_STATE  0x64300030       //�����л�ʹ�ܱ�־

#define CPU_STATE                  0x60800002			//�����״̬�Ĵ���

  
  extern void set_zero_cmd(void);

  void  FLY_Task(void *p_arg);  
  void sc_timing_off(void);
  void fly_prog(void);
  void ckzj_timing_boot(void);
  //void ckdydcz_lock(void);
  void cmd_tab_manage(void);
  void ck_reset(void);
  //void ck_antePos (void);
	void get_wk_data(void);	
	void temperature_control(void);
	void ckstate_update(void);
	void change_yc(void);
  void bubble_sort(CPU_FP32 *array,CPU_INT16U length);            
  //void batt_balance_ctrl(void);
	void batt_change(CPU_FP32 *batt_param);	
	void batt_over_protec(void);	

  CPU_FP32   batt_value(CPU_FP32 *batt_vol,CPU_INT16U num);
  


#endif
