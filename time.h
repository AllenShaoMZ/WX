/*�������޸� */

#ifndef _TIME_H_
#define _TIME_H_

#include <cpu.h>
//#include <tc.h>
#include <stdlib.h>

#define TIME_REG_ADD          0x60000200

#define SECLENGTH_INMS				0x3E8   //���ں������1000
#define WEEKLENGTHSEC         0x93A80 //���������7*86400
#define MSECOf3HOUR           0xA4CB80 //3Сʱ�ĺ�����ֵ
#define GDZQ_MS               0x00568088//�������1:34:29s


#define DAYLENGTH_INMS				0x05265c00  //���ں������86400000
#define DAYLENGTH_INSEC 			0x15180     //���������86400


#define HARD_TIME_REG         0x60000218
//#define SZJSJ_PPS_REG         0x60800004


#pragma pack(1)
typedef struct
{
    CPU_INT32U    time_s;
    CPU_INT16U    time_ms;
 }T_TIME;
#pragma pack()


#pragma pack(1)
typedef struct
{
	CPU_INT16U head;
	CPU_INT16U mode_word;   //��ʽ��
	CPU_INT16U send_lenth;  //���ͳ���
	CPU_INT16U flag;  //���ܱ�־
	CPU_INT08U att_time_sec[6];//�����̬ʱ��0.1�������
	CPU_INT08U ssztj_Y[4];//ʵʱ��̬��Y����
	CPU_INT08U ssztj_X[4];//ʵʱ��̬��X����
	CPU_INT08U ssztj_Z[4];//ʵʱ��̬��Zƫ��
	CPU_INT08U ssztj_Y_vel[4];//ʵʱ��̬���ٶ�Y����
	CPU_INT08U ssztj_X_vel[4];//ʵʱ��̬���ٶ�X����
	CPU_INT08U ssztj_Z_vel[4];//ʵʱ��̬���ٶ�Zƫ��
	
	CPU_INT16U xor;//��У��
} BROADCASTZH1;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U head;
	CPU_INT16U mode_word;   //��ʽ��
	CPU_INT16U send_lenth;  //���ͳ���
	CPU_INT16U flag;//���ܱ�־
	CPU_INT32U fixed;//�̶�ֵ	
	CPU_INT32U sat_time_sec;//����ʱ�������	
	CPU_INT16U sum;//��У��
} BROADCASTZH2;
#pragma pack()



void GNSS_R_broadcast(void);
void SBT_DCS_broadcast(void);


void entry_manage(void);
CPU_INT64U  get_time_48bit(void);
//CPU_INT64U  get_time_40bit(CPU_INT08U *time_code);
void systime_add(void);
void Now(CPU_INT32U* sec,CPU_INT16U *msec);
void Now_gdz(CPU_INT16U *OrbiCount,CPU_INT32U *msec);

void tc_cent_adjust (void);
void tc_aver_adjust(void);

void gdz_cent_adjust (void);
void gdz_aver_adjust(void);
void gdz_add(void);
void GDZQAdju (void);
void gps_ok_proce (void);
void sys_gps_erro (void);
void tc_gps_adjust (void);


#endif

