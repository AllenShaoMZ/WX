/*
 * typedef.h
 *
 *  Created on: 2023��10��13��
 *      Author: linxiaojun
 */

#ifndef SRC_HEAD_TYPEDEF_H_
#define SRC_HEAD_TYPEDEF_H_

#define MAX_PROGTAB_ITEMS 100  // shao

//*********************************************************
//                      ͷ�ļ�
//*********************************************************
#include "define.h"
#include "cpu.h" // shao

//*********************************************************
//                      �����ض���
//*********************************************************
typedef uint64_t	Uint64;
typedef uint32_t	Uint32;
typedef uint16_t	Uint16;
typedef uint8_t		Uint8;
typedef int64_t		Int64;
typedef int32_t		Int32;
typedef int16_t		Int16;
typedef int8_t		Int8;

/*shao GCZM  ZSTM*/
#pragma pack(1)
typedef struct
{
	//ע��ң�������0 ֡
	CPU_INT16U G_CLK                  ; //Z0.1   (01H)��Ӳ��ʱѡ��
	CPU_INT16U G_INJK                 ; //Z0.2   (02H)��������FLASH��־
	CPU_INT16U G_ALLOWCHECK           ; //Z0.3   (03H)��������У��
	CPU_INT16U G_ALLOWPROG            ; //Z0.4   (04H)�����ڹ��̱�־
	CPU_INT16U Payload_colle_tag      ; //Z0.5   (05H)���ݲɼ���־�����־
	CPU_INT16U adjust_mode_tag        ; //Z0.6   (06H)ϵͳ��Уʱѡ��
	CPU_INT16U gps_limit              ; //Z0.7   (07H)ϵͳ��GPSУʱ����
	CPU_INT16U gps_space              ; //Z0.8   (08H)ϵͳ��GPSУʱ���
	CPU_INT16U Syscast_tag            ; //Z0.9   (09H)ϵͳ��Ϣ�㲥׼����־
	CPU_INT16U urgent_wk_limit        ; //Z0.10  (0AH)Ӧ���¿����޿��Ʒ�Χ
	CPU_INT16U ck_reset_permit        ; //Z0.11  (0BH)�̿ظ�λ׼����־
	CPU_INT16U close_permit_tag       ; //Z0.12  (0CH)Ӧ��״̬��־
	CPU_INT16U PCDU_reset_permit      ; //Z0.13  (0DH)PCDU��λ׼����־
	CPU_INT16U sc_timing_off_permit   ; //Z0.17  (11H)������ʱ�ػ�׼����־
	CPU_INT16U wk_permit_tag          ; //Z0.49  (31H)������׼����־
	CPU_INT16U wk_select_tag          ; //Z0.50  (32H)����ͨ������ѡ���־
	
	//ע��ң�������1֡ (������)
	CPU_INT16U Hbatt_heat_limit_1     ; //Z1.16   (74H)   ��ز�������������1
	CPU_INT16U Hbatt_heat_limit_2     ; //Z1.17   (75H)   ��ز�������������2
	CPU_INT16U HdownAnt_heat_limit_1  ; //Z1.18   (76H)   GNSS���ӵ���Ų�������������1
	CPU_INT16U HdownAnt_heat_limit_2  ; //Z1.19   (77H)   GNSS���ӵ���Ų�������������2
	CPU_INT16U HdownAnt_heat_limit_3  ; //Z1.20   (78H)   GNSS���ӵ���Ų�������������3
	CPU_INT16U HupAnt_heat_limit      ; //Z1.21   (79H)   GNSS���ӵ���Ų�������������
	CPU_INT16U HZH1M_heat_limit       ; //Z1.49   (95H)		һ���������������1����
	CPU_INT16U HZH1B_heat_limit       ; //Z1.50   (96H)		һ���������������1����
	CPU_INT16U HXKZM_heat_limit       ; //Z1.51  (97H)		���������������������
	CPU_INT16U HXKZB_heat_limit       ; //Z1.52   (98H)		����󱸲�������������
	CPU_INT16U HZH2M_heat_limit       ; //Z1.53   (99H)		һ���������������2����
	CPU_INT16U batt_protect_switch    ; //Z1.54   (9AH)		���ű������ܱ�־
	CPU_INT32U CKZJ_TIMING_time       ; //Z1.55-56(9B-9CH)		�������1��λʱ������
	CPU_INT16U HZH2B_heat_limit	      ; //Z1.57   (9DH)		һ���������������2����
	CPU_INT16U HXM1_heat_limit	      ; //Z1.58   (9EH)		����1��������������
	CPU_INT16U HXM2_heat_limit	      ; //Z1.59   (9FH)		����2��������������
	CPU_INT16U HPCDU_heat_limit       ; //Z1.60   (A0H)		PCDU��������������
	
  // ע��ң�������1֡ (������)

    CPU_INT16U inertialUnit_heat_limit; // �������������
    CPU_INT16U starSensor_heat_limit  ; // ��������������
    CPU_INT16U cage_heat_limit        ; // �������������
    CPU_INT16U coolThrust1_limit      ; // ����������1����������
    CPU_INT16U coolThrust2_limit      ; // ����������2����������
    CPU_INT16U coolBottle_limit       ; // ������ƿ����������
    CPU_INT16U coolValve_limit        ; // ������ѹ������������
    CPU_INT16U catalyticBed1_limit    ; // ����Ԫ�߻���1����������
    CPU_INT16U catalyticBed3_limit    ; // ����Ԫ�߻���3����������
    CPU_INT16U catalyticBed5_limit    ; // ����Ԫ�߻���5����������
    CPU_INT16U catalyticBed6_limit    ; // ����Ԫ�߻���6����������
    CPU_INT16U catalyticBed7_limit    ; // ����Ԫ�߻���7����������
    CPU_INT16U catalyticBed8_limit    ; // ����Ԫ�߻���8����������
    CPU_INT16U fuelTank_limit         ; // ����Ԫ�������������
    CPU_INT16U fuelLine_limit         ; // ����Ԫ��·����������
    CPU_INT16U pressureSensor_limit   ; // ѹ������������������
    CPU_INT16U solenoidValve1_limit   ; // ��ŷ�1����������
    CPU_INT16U solenoidValve2_limit   ; // ��ŷ�2����������
    CPU_INT16U observationUnit_limit  ; // �����������������
    CPU_INT16U ZDB_limit              ; // ZDB����������
    CPU_INT16U batteryTemp_limit      ; // �����¶�״̬����������
		
		// �洢ģʽ�µ�����
    CPU_INT16U inertialUnit_storage_limit;  // ����������洢ģʽ����
    CPU_INT16U starSensor_storage_limit;    // �����������洢ģʽ����
    CPU_INT16U cage_storage_limit;          // ����������洢ģʽ����
    CPU_INT16U coolThrust1_storage_limit;   // ����������1�洢ģʽ����
    CPU_INT16U coolThrust2_storage_limit;   // ����������2�洢ģʽ����
    CPU_INT16U coolBottle_storage_limit;    // ������ƿ�洢ģʽ����
    CPU_INT16U coolValve_storage_limit;     // ������ѹ���洢ģʽ����
    CPU_INT16U catalyticBed1_storage_limit; // ����Ԫ�߻���1�洢ģʽ����
    CPU_INT16U catalyticBed3_storage_limit; // ����Ԫ�߻���3�洢ģʽ����
    CPU_INT16U catalyticBed5_storage_limit; // ����Ԫ�߻���5�洢ģʽ����
    CPU_INT16U catalyticBed6_storage_limit; // ����Ԫ�߻���6�洢ģʽ����
    CPU_INT16U catalyticBed7_storage_limit; // ����Ԫ�߻���7�洢ģʽ����
    CPU_INT16U catalyticBed8_storage_limit; // ����Ԫ�߻���8�洢ģʽ����
    CPU_INT16U fuelTank_storage_limit;      // ����Ԫ����洢ģʽ����
    CPU_INT16U fuelLine_storage_limit;      // ����Ԫ��·�洢ģʽ����
    CPU_INT16U pressureSensor_storage_limit;// ѹ���������洢ģʽ����
    CPU_INT16U solenoidValve1_storage_limit;// ��ŷ�1�洢ģʽ����
    CPU_INT16U solenoidValve2_storage_limit;// ��ŷ�2�洢ģʽ����
    CPU_INT16U observationUnit_storage_limit;// ��������洢ģʽ����
    CPU_INT16U ZDB_storage_limit;           // ZDB�洢ģʽ����
    CPU_INT16U batteryTemp_storage_limit;   // �����¶�״̬�洢ģʽ����
	
	//ע��ң�������2 ֡ (�ɱ�̺�����)

  CPU_INT32U ramtab[MAX_PROGTAB_ITEMS];
	
} ZSTM;

#pragma pack(1)
typedef struct
{
	// ����ң�������0֡
	CPU_INT16U dir_cmd_count         ;  //G0.1     ֱ��ָ�����
	CPU_INT16U indir_cmd_count       ;  //G0.2     ���ָ�����
	CPU_INT16U inn_cmd_count         ;  //G0.3     �ڲ�ָ�����
	CPU_INT16U soft_zs_count         ;  //G0.4     ���ע������
	CPU_INT16U payload_zs_count      ;  //G0.5     ҵ��ע������
	CPU_INT16U read                  ;  //G0.6     ָ����ж�ָ��
	CPU_INT16U write                 ;  //G0.7     ָ�����дָ��
	CPU_INT16U enter_200ms           ;  //G0.8     200ms��ڼ���
	CPU_INT16U enter_400ms           ;  //G0.9     400ms��ڼ���
	CPU_INT16U enter_1s              ;  //G0.10    1s��ڼ���
	CPU_INT16U enter_2s              ;  //G0.11    2s��ڼ���
	CPU_INT16U enter_5s              ;  //G0.12    5s��ڼ���
	CPU_INT16U enter_1m              ;  //G0.13    1����ڼ���
	CPU_INT16U prog_state            ;  //G0.14    ��������״̬
	CPU_INT16U comp_rule_state       ;  //G0.15    �����״̬��־
	CPU_INT16U yc_status3            ;  //G0.28	  ң��״̬ң��3
	CPU_INT16U syn_ok_cnt            ;  //G0.29   ͬ��ͷ���ϼ���
	CPU_INT16U heater_state          ;  //G0.36    ������״̬��־
	CPU_INT16U vcid_ok_cnt           ;  //G0.43VCID���ϼ���
	CPU_INT32U ckzj_timing_count     ;  //G0.69_70  �м̲�ض�ʱ����
	CPU_INT16U frame_ok_cnt          ;  //G0.71 ֡��ʽ���ϼ���
	CPU_INT16U first_power_tag       ;  //G0.85�״��ϵ��־��
	
	// ����ң�������2֡(��ע��ң���2֡���ƣ�//

	CPU_INT32U TeYK                    ; //G1.1_2      ϵͳ�Ӿ���Уʱ��
	CPU_INT32U GDZeYk                  ; //G1.3_4      ����Ӿ���Уʱ��
	CPU_INT32U SoftTime_S              ; //G1.5_6      ϵͳ��ʱ�������
	CPU_INT16U SoftTime_mS             ; //G1.7        ϵͳ��ʱ���������(ms)
	CPU_INT16U OrbiCount               ; //G1.8        ���Ȧ��
	CPU_INT32U GDZs                    ; //G1.9_10     �������ʱ��(ms)
	CPU_INT32S DeltaGDZ                ; //G1.11_12    ����ӵļ���Уʱ��
	CPU_INT32S Delta_SysTime_s         ; //G1.13_14    ϵͳ�Ӽ���Уʱ�������
	CPU_INT16S Delta_SysTime_ms        ; //G1.15      ϵͳ�Ӽ���Уʱ���������
	CPU_INT16U Tm_baud                 ; //G1.24   (7CH)   ң���������л�20170904�޸�
	CPU_INT16U Tm_down_tag             ; //G1.29   (81H)   ң���´���ʽ��
	CPU_INT16U autowk_permit_tag       ; //G1.39      �Զ��¿������־
	CPU_INT16U Zkfault                 ; //G1.40      wxy��������ŵ��־����3�׶�
	CPU_INT16U first_order_Vth         ; //G1.42    ���ŵ籣�����������ѹ����V1(һ�׶����ص�ѹ��ֵ)(10.8V)
	CPU_INT16U second_order_Vth        ; //G1.43    ���ŵ籣���������ѹ����V2(���׶����ص�ѹ��ֵ)(10.5V)
	CPU_INT16U third_order_Vth        ; //G1.44    ���ŵ籣���������ѹ����V3(���׶����ص�ѹ��ֵ)(10.2V)
	CPU_INT16U batts_Vth               ; //G1.45    �������ѹ��ֵ(10.8V)
	CPU_INT16U batt_protect_switch    ; //G1.54   (9AH)		���ű������ܱ�־
	CPU_INT16U energy_management_tag   ; //G1.63	��Դ����׼��
	CPU_INT16U auto_connect_enable_state; //G1.64	�ŵ翪��������ͨʹ�ܱ�־λ
	CPU_INT16U current_out_Vth         ; //G1.66   ���ŵ籣�����طŵ��������V8(0.5A)
	CPU_INT16U batts_temp_Vth          ; //G1.67   �ŵ翪��������ͨ�����¶���ֵ(0��)
} GCTM;
#pragma pack()

// shao ����ָ�����
#pragma pack(1)
typedef struct					//����ָ�����
{
    uint32_t addr_on;   // ��ַ��32λ��
    uint16_t cmd_on;    // �����16λ��
    uint32_t addr_off;  // ��ַ��32λ��
    uint16_t cmd_off;   // �����16λ��
}CMD_Q;
#pragma pack()

/*���״̬*/
typedef struct ZJ_STATE
{
	uint8_t			ucGMCan;						//��ЧGM Can
	uint8_t			ucTXJCan;						//��ЧTXJ Can
	uint8_t			ucWX_YCMode;					//����ң��ģʽ
	uint8_t			ucSecondFlag;					//����
	uint8_t			ucSecondCnt;					//�����
	Uint32			uiPingPang;						//ƹ��
	Uint32			auiImgSendDataReady[2];			//������ͼ������׼���ñ�ǣ�[0]����ǰ��[1]�����
	Uint8			ucImageReturnState;				//ͼ��ش�״̬
	Uint8			ucSZProgramCRCState;			//���鴦�������ע����У��״̬
	Uint8			ucProgramSXState;				//���鴦�����������д���״̬
	Uint32			uiZDBAttackTime;				//zdb���ʱ��
	Uint8			ucCameraReloadFlag;				//������ر�־
	Uint8			ucCameraBootFlag;				//���������־��0,��ʾδ������δ�������ò�������1��ʾ�������������2��ʾ���������3sʹ��FPGAͼ����3��ʾ���������ң��
	Uint8			ucDEMBootFlag;					//����������־��0��ʾδ�ϵ���δ��ʱ��1��ʾ�����ϵ���δ��ʱ��2��ʾ�����ϵ�����ʱ��3��ʾ����������ң��
	Uint8			ucSpacecraftIDFlag;				//������ʶ�����־��0��ʾû�к�����ʶ���룬1��ʾ�к�����ʶ����
	Uint32			uiSecondCnt;					//�������������
	Uint8			ucXMRecvFlag;					//�������ݽ����ж�
	Uint8			ucGNSSNewFlag;					//GNSS���ݽ����ж�
	Uint8			ucZSSFlag;						//����ʱ��־
	Uint32			uiUTCSecond;					//UTCʱ�䣬��
	Uint32			uiUTCMicroSecond;				//UTCʱ�䣬΢��
	Uint8			ucBCEnable;						//�㲥ʹ�ܱ��
	Uint8			ucZDBBootFlag;					//ս����������־
	Uint32			uiZDBSelfCheckTime;				//ZDB�Լ�ʱ��ms
	Uint32			uiSelfCheckTime;				//�Լ�ʱ��s
	Uint8			ucSelfCheckFlag;				//�Լ���
	Uint8			ucSelfCheckState;				//�Լ�״̬
	Uint32			uiGPSUTCSecond;
	Uint32			uiGPSUTCMicroSecond;
	float			fLat;
	float			fLon;
	float			fHeight;
}T_ZJ_STATE;

/*��1�������0���㷨���н��״̬*/
typedef struct ALG_STATE
{
	Uint8			ucImgGrayAvg;
	Uint8			ucImgGrayStd;
		
	Uint8			ucGMSoftWareMode;
		
	Uint8			ucRegionNumState;
	Uint8			ucRegionNum:4;					//B7~B4����ֵ�������ͨ������
	Uint8			ucDelTargetNum:4;				//B3~B0���޳��龯��Ŀ������
		
	Uint8			ucImgTargetCaptureState;
	Uint8			ucCameraOverexposure:2;			//B7-B6���������״̬
	Uint8			uc1QuadrantStrongLight:1;		//B5����һ����ǿ���־
	Uint8			uc2QuadrantStrongLight:1;		//B4���ڶ�����ǿ���־
	Uint8			uc3QuadrantStrongLight:1;		//B3����������ǿ���־
	Uint8			uc4QuadrantStrongLight:1;		//B2����������ǿ���־
	Uint8			ucTargetCaptureMode:1;			//B1��Ŀ�겶��ʽ
	Uint8			ucTargetCaptureFlag:1;			//B0��Ŀ�겶���־

	Uint8			ucStarState;
	Uint8			ucTriangularMatchFlag:4;		//����ƥ���ж�
	Uint8			ucXMDataRecvFlag:2;				//�������ݽ����ж�
	Uint8			ucStarDelFlag:2;				//�����޳��ж�

	float			fRA;							//�������ָ�������ྭ���
	float			fDEC;							//�������ָ��������γ���
	Uint8			ucTargetFlightFlag;				//Ŀ��켣�����ж�
	Uint8			ucTargetPosFlag;				//Ŀ��λ���ж�
	Uint16			usTargetBright;					//Ŀ���������
	Uint16			usTargetShape;					//Ŀ����״���
	Uint32			uiTargetSize;					//Ŀ��������
	Uint8			ucTargetConfidence;				//Ŀ�����Ŷ����
	float			fTargetAZ;						//Ŀ��ͶӰ�Ǧ���tanֵ���
	float			fTargetEL;						//Ŀ��ͶӰ�Ǧµ�tanֵ���
}T_ALG_STATE;

/*��1�������0��λ�˽�����Ϣ*/
typedef struct GESTURE_INFO
{
	/*����λ�˽������Ϣ*/
	Uint32			*puiBinImg;						//��λ�˽����ֵ��ͼ���ַ
	Uint16			usXCentroid;					//Ŀ��X��������
	Uint16			usYCentroid;					//Ŀ��Y��������
	Uint8			ucGestureFlag;					//λ�˼�����
}T_GESTURE_INFO;

/*��1�������0�Ľ�����ͼ����Ϣ*/
typedef struct DOWN_IMG_INFO
{
	Uint8			ucDownImgType;					//������ͼ�����ͣ�1������ǰ��2�������
}T_DOWN_IMG_INFO;

/*��1�������0����Ϣ*/
typedef struct CORE1TO0_SHARE_INFO
{
	/*״̬����*/
	Uint8			ucType;							//0x11��ֻ���㷨���н��״̬��0x22������λ�ˣ�0x33������������ͼ����Ϣ��0x44��������

	/*�㷨���н��״̬*/
	T_ALG_STATE 	tAlgState;

	/*λ����Ϣ*/
	T_GESTURE_INFO 	tGestureInfo;

	/*������ͼ����Ϣ*/
	T_DOWN_IMG_INFO	tDownImgInfo;
}T_CORE1TO0_SHARE_INFO;

/*��0�������1����Ϣ*/
typedef struct CORE0TO1_SHARE_INFO
{
	Uint8			ucType;							//������Ϣ����
	/*ָ��*/
	Uint8			ucImageReturnApply;				//ͼ��ش�����ָ��				0x11
	Uint8			ucGMMode;						//�������ģʽ(��ͬ��״Ŀ��)	0x12
	Uint8			ucImgReverse;					//ͼ��ת						0x13
	/*֪ͨ��1�µ���������*/
	float			fquatenion0;					//��Ԫ��0   					0x22
	float			fquatenion1;					//��Ԫ��1
	float			fquatenion2;					//��Ԫ��2
	float			fquatenion3;					//��Ԫ��3
	/*֪ͨ��1�ķ����ͼ��ɼ����*/
	Uint8			aucImageStoreFlag[4];			//�����ͼ��ɼ���־			0x33

}T_CORE0TO1_SHARE_INFO;

/*�ǵ���ͨ��Ϣ*/
typedef struct CORNER_LINK
{
	Uint16 			usXOrd;							//�ǵ�x��������
	Uint16 			usYOrd;							//�ǵ�y��������
	Uint16			usLinkCnt8;						//�ǵ�8��ͨ����
	Uint16			usUnLinkCnt24;					//�ǵ�24����ͨ����
}T_CORNER_LINK;

/*��ͨ����Ϣ*/
typedef struct REGION_INFO
{
	Uint32 			uiRegionSize;					//��ͨ���С
	Uint16			usEdgeLen;						//��Ե����
	Uint16			usCornerLen;					//�ܽǵ���
	T_CORNER_LINK 	*ptCornerLink;					//�ǵ�������Ϣ
}T_REGION_INFO;

/*��0λ�˽���״̬*/
typedef struct GESTURE_STATE
{
	Uint8			ucTargetDisState;				//Ŀ������״̬
	Uint16			usTargetDis;					//Ŀ���������1
	Uint8			ucTargetDis;					//Ŀ���������2
	float			fTargetXGesture;				//Ŀ��X����̬
	float			fTargetYGesture;				//Ŀ��Y����̬
	float			fTargetZGesture;				//Ŀ��Z����̬
}T_GESTURE_STATE;

/*�ǵ���Ϣ*/
typedef struct CORNER_INFO
{
	Uint16 			usXOrd;							//�ǵ�x��������
	Uint16 			usYOrd;							//�ǵ�y��������
	Uint16			usDescripNum;					//�ǵ�����������
	Uint8			*pucDescrip;					//�ǵ�������
}T_CORNER_INFO;

/*ƥ���������2D��Ϣ*/
typedef struct POINT_2D
{
	float 			fPointXord;						//x��������
	float 			fPointYord;						//y��������
	Uint8			ucFlag;							//������ѡ����
}T_POINT_2D;

/*ƥ���������3D��Ϣ*/
typedef struct POINT_3D
{
	float 			fPointXord;						//x��������
	float 			fPointYord;						//y��������
	float 			fPointZord;						//z��������
}T_POINT_3D;

/*����*/
typedef struct COMPLEX
{
	double 			dR;								//ʵ��
	double 			dI;								//�鲿
}T_COMPLEX;



#endif /* SRC_HEAD_TYPEDEF_H_ */
