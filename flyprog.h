#ifndef _FLYPROG_H_
#define _FLYPROG_H_

#include <math.h>

#include "os.h"  


//#define CKZJDIRCMDCNT  0x60400002         //测控中继
#define DIRCMDCNT     0x60200002       //扩频应答机直接指令计数寄存器
#define SYN_OK_CNT    0x60200006       //同步头符合计数
#define VCID_OK_CNT   0x60200008       //VCID符合计数
#define YC_STATUS3   0x6020000A       //遥控状态遥测3
//Bit3~bit0：虚拟信道识别信息
//bit4：数据来源，0：地面，1：加解密
//bit5：BCH译码状态，0：正确，1：错误
//bit6：CRC校验状态，0：正确，1：错误
//bit7：遥控转明状态，1：正常，0：快转明
#define FRAME_OK_CNT  0x6020000E       //帧格式符合计数
#define SECOf1WEEK   0x93A80 			//1周的秒数值
#define SECOf25MIN   0x05DC 			//25min的秒数值

#define HEATER_ROADNUM 21               //加热器路数


#define AUTO_CONNECT_ENABLE_STATE 0x64300000 //自主接通使能标志
#define ZK_START  0x64300010           //姿控软件启动标志
#define FIRST_POWER  0x64300020       //卫星首次上电标志
#define AUTO_SWITCH_ENABLE_STATE  0x64300030       //自主切机使能标志

#define CPU_STATE                  0x60800002			//计算机状态寄存器

  
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
