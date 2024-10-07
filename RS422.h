#ifndef _RS422_H_
#define _RS422_H_

#include  <cpu.h>
//#include  <serial.h>
#include  <os.h>
#include  "memory.h"
#include "typedef.h"
//#include  "../AOCS/aocs-def-pubconst.h"
#include  <stdbool.h>

/*-------------------------------------------------------*/
#define	GNSS_LEN	(sizeof(GNSS_TM)+2)
#define	GNB_LEN	(sizeof(GNB_TM)+2)
#define	XM_LEN	(sizeof(XM_TM)+2)
#define	FL_LEN	(sizeof(FL_TM)+1)
#define	PCDU_LEN	(sizeof(PCDU_TM)+4)
#define	SJTX_LEN	(sizeof(T_BkPkg)+1)
#define	EP_LEN	(sizeof(EP_TM)+2)
#define	TL_LEN	(sizeof(TL_TM))
#define	GNSS_R_LEN	(sizeof(GNSS_R_TM)+2)
#define	SBT_DCS_LEN	(sizeof(SBT_DCS_TM)+2)
#define	XTTC_LEN	(sizeof(XTTC_TM)+2)


#define	DLX_TM_LEN	(sizeof(DLX_TM)+2)
#define	AOCC_TM_LEN	(sizeof(AOCC_TM))

#define UTC2BJT		(8*3600*1000) 

#define	GNSS_R_CMD_ERR	0x79
#define	XM_CMD_ERR	0xAA

//���͸�λ����ַ
#define	GNSS_SRST_BASE	0x649FE0C0	//GNSS����
#define	XMA_SRST_BASE	0x649FE0C4	//����1
#define	ZH3_SRST_BASE	0x649FE0C8	//�غ�3
#define	XMB_SRST_BASE	0x649FE0CC	//����2
#define	ZH2_SRST_BASE	0x649FE0D0	//�غ�2 GNSS_R
#define	TLA_SRST_BASE	0x649FE0D4	//����A
#define	TLB_SRST_BASE	0x649FE0D8	//����B
#define	FLX_SRST_BASE	0x649FE0DC	//����1
#define	FLY_SRST_BASE	0x649FE0E0	//����2
#define	FLZ_SRST_BASE	0x649FE0E4	//����3
#define	PCDU_SRST_BASE	0x649FE0E8	//PCDU
#define	EP_SRST_BASE	0x649FE0EC	//����
#define	FLS_SRST_BASE	0x649FE0F0	//�������
#define	SJTX_SRST_BASE	0x649FE0F4	//˫��ͨ��
#define	DLX_SRST_BASE	0x649FE0F8	//����ѧ
#define	GNB_SRST_BASE	0x649FE0FC	//���ܰ�
#define	ZH1_SRST_BASE	0x649FE8C4  //�غ�1 SBT_DCS
#define	CKYK_SRST_BASE	0x649FE8CC	//���ң��



//���ͻ������ַ
#define	GNSS_SCACHE_BASE	0x64800000	//GNSS����
#define	XMA_SCACHE_BASE		0x64800400	//����1
#define	ZH3_SCACHE_BASE		0x64800800	//�غ�3
#define	XMB_SCACHE_BASE		0x64800C00	//����2
#define	ZH2_SCACHE_BASE		0x64801000	//�غ�2
#define	TLA_SCACHE_BASE		0x64801400	//����A
#define	TLB_SCACHE_BASE		0x64801800	//����B
#define	FLX_SCACHE_BASE		0x64801C00	//����1
#define	FLY_SCACHE_BASE		0x64802000	//����2
#define	FLZ_SCACHE_BASE		0x64802400	//����3
#define	PCDU_SCACHE_BASE	0x64802800	//PCDU
#define	EP_SCACHE_BASE		0x64802C00	//����
#define	FLS_SCACHE_BASE		0x64803000	//�������
#define	SJTX_SCACHE_BASE	0x64803400	//˫��ͨ��
#define	DLX_SCACHE_BASE		0x64803800	//����ѧ
#define	GNB_SCACHE_BASE		0x64803C00	//���ܰ�
#define	ZH1_SCACHE_BASE		0x64814400  //�غ�1
#define	CKYK_SCACHE_BASE	0x64814C00	//���ң��


//���ͳ��Ȼ���ַ��������
#define	GNSS_SLEN_BASE	0x649FE000	//GNSS����
#define	XMA_SLEN_BASE	0x649FE004	//����1
#define	ZH3_SLEN_BASE	0x649FE008	//�غ�3
#define	XMB_SLEN_BASE	0x649FE00C	//����2
#define	ZH2_SLEN_BASE	0x649FE010	//�غ�2
#define	TLA_SLEN_BASE	0x649FE014	//����A
#define	TLB_SLEN_BASE	0x649FE018	//����B
#define	FLX_SLEN_BASE	0x649FE01C	//����1
#define	FLY_SLEN_BASE	0x649FE020	//����2
#define	FLZ_SLEN_BASE	0x649FE024	//����3
#define	PCDU_SLEN_BASE	0x649FE028	//PCDU
#define	EP_SLEN_BASE	0x649FE02C	//����
#define	FLS_SLEN_BASE	0x649FE030	//�������
#define	SJTX_SLEN_BASE	0x649FE034	//˫��ͨ��
#define	DLX_SLEN_BASE	0x649FE038	//����ѧ
#define	GNB_SLEN_BASE	0x649FE03C	//���ܰ�
#define	ZH1_SLEN_BASE	0x649FE804  //�غ�1
#define	CKYK_SLEN_BASE	0x649FE80C	//���ң��


//���ջ������ַ
#define	GNSS_RCACHE_BASE	0x64804000	//GNSS����
#define	XMA_RCACHE_BASE		0x64805000	//����1
#define	ZH3_RCACHE_BASE		0x64806000	//�غ�3
#define	XMB_RCACHE_BASE		0x64807000	//����2
#define	ZH2_RCACHE_BASE		0x64809000	//�غ�2
#define	TLA_RCACHE_BASE		0x64809400	//����A
#define	TLB_RCACHE_BASE		0x64809800	//����B
#define	FLX_RCACHE_BASE		0x64809C00	//����1
#define	FLY_RCACHE_BASE		0x6480A000	//����2
#define	FLZ_RCACHE_BASE		0x6480A400	//����3
#define	PCDU_RCACHE_BASE	0x6480A800	//PCDU
#define	EP_RCACHE_BASE		0x6480AC00	//����
#define	FLS_RCACHE_BASE		0x6480B000	//�������
#define	SJTX_RCACHE_BASE	0x6480B400	//˫��ͨ��
#define	DLX_RCACHE_BASE		0x6480B800	//����ѧ
#define	GNB_RCACHE_BASE		0x6480BC00	//���ܰ�
#define	ZH1_RCACHE_BASE		0x6480D000  //�غ�1
#define	CKAZGBC_RCACHE_BASE	0x6480E000	//���A�ڹ���
#define	CKBZGBC_RCACHE_BASE	0x6480F000	//���B�ڹ���
#define	CKBYC_RCACHE_BASE	0x64811000	//���Bң��
#define	CKAYC_RCACHE_BASE	0x64811400	//���Aң��


//���ո�������ַ����λ��
#define	GNSS_RLEN_BASE		0x649FE400	//GNSS����
#define	XMA_RLEN_BASE		0x649FE404	//����1
#define	ZH3_RLEN_BASE		0x649FE408	//�غ�3
#define	XMB_RLEN_BASE		0x649FE40C	//����2
#define	ZH2_RLEN_BASE		0x649FE410	//�غ�2
#define	TLA_RLEN_BASE		0x649FE414	//����A
#define	TLB_RLEN_BASE		0x649FE418	//����B
#define	FLX_RLEN_BASE		0x649FE41C	//����1
#define	FLY_RLEN_BASE		0x649FE420	//����2
#define	FLZ_RLEN_BASE		0x649FE424	//����3
#define	PCDU_RLEN_BASE		0x649FE428	//PCDU
#define	EP_RLEN_BASE		0x649FE42C	//����
#define	FLS_RLEN_BASE		0x649FE430	//�������
#define	SJTX_RLEN_BASE		0x649FE434	//˫��ͨ��
#define	DLX_RLEN_BASE		0x649FE438	//����ѧ
#define	GNB_RLEN_BASE		0x649FE43C	//���ܰ�
#define	ZH1_RLEN_BASE		0x649FEC04  //�غ�1
#define	CKAZGBC_RLEN_BASE	0x649FEC08	//���A�ڹ���
#define	CKBZGBC_RLEN_BASE	0x649FEC0C	//���B�ڹ���
#define	CKBYC_RLEN_BASE		0x649FEC10	//���Bң��
#define	CKAYC_RLEN_BASE		0x649FEC14	//���Aң��

#define DLX_422_BASE	0x60084000       //DLX

#define OFFSET_ADDR     0x2000

/*-----------------������ַ����----------------------*/
#define INDIR_ADDR					0x00     //���ָ��

#define PROCESS_BOARD_ADDR			0x61     //��������
#define ZK_MODULE_ADDR				0x62     //�˹��ģ��
#define	PCDU_ADDR					0x63	//��Դ������
#define GNB_ADDR					0x64     //���ܰ�
#define GPS_ADDR					0x65     //GPSģ��
#define XTTCA_ADDR					0x66     //�������һ���A
#define XTTCB_ADDR					0x67     //�������һ���B
#define XJTX_ADDR					0x68     //�Ǽ�ͨ��
#define	DZB_ADDR					0x69	//���ذ�
#define	XMA_ADDR					0x6a	//����A
#define	XMB_ADDR					0x6b	//����B
#define	FLX_ADDR					0x6c	//����X
#define	FLY_ADDR					0x6d	//����Y
#define	FLZ_ADDR					0x6e	//����Z
#define	XKZ_ADDR					0x6f	//�����
#define	EP_ADDR						0x70	//����
#define	FLS_ADDR					0x71	//����S
#define	GSA_ADDR					0x72	//����A
#define	GSB_ADDR					0x73	//����B
#define	ZH1_ADDR					0x74	//�غ�1
#define	ZH2_ADDR					0x75	//�غ�2
#define	ZH3_ADDR					0x76	//�غ�3
#define	DLX_ADDR					0x77	//����ѧ


/*----------------------------------------------------*/


enum RS422_MACHINE {XTTCA,XTTCB,TLA,TLB,GNSS,GNB,XMA,XMB,FLX,FLY,FLZ,FLS,PCDU,EP,GNSS_R,SBT_DCS,SJTX};
enum RS422_MACHINE2 {BAK1,BAK2,AOCC,DLX};

//CKJJM����USBYDJ����
//wxy:���޸�.����ö�����ʹ�����.�Ƿ���˳������?

#pragma pack(1)
typedef struct
{
	CPU_INT16U send_num;
	CPU_INT16U rec_num;
	CPU_INT16U send_start;
	CPU_INT16U baud_set;
	CPU_INT16U parity;
} RS422_CTRL;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U* dev_chan;
	RS422_CTRL* rs422_ctrl_reg;
	CPU_INT32U rec_lenth;
} RS422_COLL;
#pragma pack()

//RS422���ջ���
#pragma pack(1)
typedef struct
{
	CPU_INT16U* sendrst;
	CPU_INT16U* sendbase;
	CPU_INT16U* sendlenbase;
	//CPU_INT16U	sendlen;
	
	CPU_INT16U* recvbase;
	CPU_INT16U* recvlenbase;
	CPU_INT16U	recvlen;
} RS422_RECV;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT32U flRealV;		//���ֵ�ǰʵ��ת��
	CPU_INT32U flRealCur;	//���ֵ�ǰʵ�ʲ�������
	CPU_INT32U ctrlStat;	//����״̬
	CPU_INT32U accelerateTorque;	//���ּ�������
	CPU_INT08U ID;			//����ָ�����
	CPU_INT08U sum;			//�ۼӺ�
} FL_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U	APID;
	CPU_INT16U	flag;
	CPU_INT16U	screenGridVol;	//6-7��դ��ѹU16
	CPU_INT16U	screenGridCur;	//8-9��դ����U16
	CPU_INT16U	accelerateGridVol;	//10-11����դ��ѹU16
	CPU_INT16U	accelerateGridCur;	//12-13����դ����U16
	CPU_INT16U	heaterVol;	//14-15��������ѹU16
	CPU_INT16U	heaterCur;	//16-17����������U16
	CPU_INT16U	neutralizerVol;	//18-19�к�����ѹU16
	CPU_INT16U	neutralizerCur;	//20-21�к�������U16
	CPU_INT08U	microwaveSrcStat;	//22΢��Դ״̬
	CPU_INT16U	pressureSensor_1;	//23-24ѹ��������1U16
	CPU_INT16U	pressureSensor_2;	//25-26ѹ��������2U16
	CPU_INT16U	pressureSensor_3;	//27-28ѹ��������3U16
	CPU_INT08U	mode;	//29����ģʽ
	CPU_INT08U	selfCheck;	//30�Լ�
	CPU_INT08U	faultFeedback;	//31���Ϸ���
} EP_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
    CPU_INT16U  APID;       //2-3APID
    CPU_INT08U  sampling;       //4ԭʼ���ݲ������أ��ƺ�Ĭ��Ϊ0��
    //CPU_INT08U  mode;           //�غɹ���ģʽ
    CPU_INT08U  stateMachine;   //5FLASH���Ƶ�״̬�������
    //CPU_INT08U  OC;             //DSP��OC�л���Ӧ
    CPU_INT16U   time[3];       //6-11GNSSʱ�䣨GNSS-R�غ����ɣ�U16[3]
    CPU_INT08U   X[4];          //12-15GNSS��λλ��X��GNSS-R�غ����ɣ�S32
    CPU_INT08U   Y[4];          //16-19GNSS��λλ��Y��GNSS-R�غ����ɣ�S32
    CPU_INT08U   Z[4];          //20-23GNSS��λλ��Z��GNSS-R�غ����ɣ�S32
    CPU_INT08U   Vx[4];         //24-27GNSS��λ�ٶ�Vx��GNSS-R�غ����ɣ�S32
    CPU_INT08U   Vy[4];         //28-31GNSS��λ�ٶ�Vy��GNSS-R�غ����ɣ�S32
    CPU_INT08U   Vz[4];         //32-35GNSS��λ�ٶ�Vz��GNSS-R�غ����ɣ�S32
    CPU_INT08U  satCnt;         //36�ɼ���������
    //CPU_INT08U  status;         //��λ��־��������Ч��־��
    //CPU_INT08U  pps;            //1PPS�ź�
    CPU_INT08U reserve[3];        //37-39Flash�������Ԥ����ȫ0��
} GNSS_R_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U	APID;	//2-3APID
	CPU_INT08U	channel_HighSpeedBit;	//4ͨ��״̬������λͬ����
	CPU_INT08U	highSpeedChannelSNR;	//5�����ŵ�SNR��
	CPU_INT08U	frequency1SNR;	//6Ƶ��1ͨ��1-SNR��
	CPU_INT08U	frequency2SNR;	//7Ƶ��2ͨ��1-SNR��
	CPU_INT08U	importantParamBackupCnt;	//8������Ҫ�������ݼ�����
	CPU_INT16U	fixedMemoryRecordAddr;	//9-10�̴��¼��ַ
	CPU_INT16U	fixedMemoryPlaybackAddr;	//11-12�̴�طŵ�ַ
	CPU_INT08U	highSpeedAGC;	//13����ͨ��AGC
	CPU_INT08U	lowSpeedAGC;	//14����ͨ��AGC
	CPU_INT08U	operatingCur;	//15��������U8
	CPU_INT08U	pwrAmplifierCur;	//16���ŵ���U8
	CPU_INT08U	operatingVol;	//17������ѹU8
	CPU_INT08U	cpuVer;	//18����������汾�ŵ�
	CPU_INT08U	generalMsgRxCnt;	//19��ͨ���Ľ��ռ�����
	CPU_INT08U	ephemerisRxCnt;	//20������ע���ռ�����
	CPU_INT08U	cpuHeartbeatCnt;	//21����������������
	CPU_INT08U	lowSpeedInjectionCnt;	//22������ע������
	CPU_INT08U	curFPGAProgramLoc;	//23��ǰFPGA����λ�õ�
	CPU_INT08U	internalBroadcastDownCnt;	//24�ڲ�״̬���㲥���м�����
	CPU_INT08U	invalid422Cnt;	//25��Ч422ָ�������
	CPU_INT08U	businessDownExecutionCnt;	//26ҵ������ִ�м�����
	CPU_INT08U	highSpeedRxCnt;	//27���ٱ��Ľ��ռ���
	CPU_INT08U	fixedMemory422Playback;	//28A�̴�422˳��طſ�/�ص�
	CPU_INT08U	businessRxCnt;	//29ע��ҵ����ռ���
	CPU_INT08U	starTime;	//30����ʱ��
	CPU_INT08U	refactorMode;	//31�ع�ģʽ��
	CPU_INT08U	curRefactorAddr_15_8;	//32��ǰ�ع���ַ[15:8]
	CPU_INT08U	curRefactorAddr_7_0;	//33��ǰ�ع���ַ[7:0]
	CPU_INT08U	refactorAreaTotEraseCnt;	//34�ع���ȫ����������
	CPU_INT08U	CRC_BlockChkExecutionCnt;	//35CRC�����������У��ִ�м�����
	CPU_INT08U	operationAddrErrCnt;	//36������ַ���������
	CPU_INT08U	FLASHFrameHeadCorrCnt;	//37�ض�FLASH֡ͷ��ȷ������
	CPU_INT16U	refactorWrFLASHSucCnt;	//38-39�ع�����дFLASH�ɹ�����
	CPU_INT08U	highSpeedInjectionRxCnt;	//40������ע���ռ���
	CPU_INT08U	curRefactorAreaNum;	//41״̬�������͡���ǰ�ع�����ŵ�
	CPU_INT08U	refactorStat_1;	//42�ع�״̬1
	CPU_INT08U	refactorStat_2;	//43�ع�״̬2
	CPU_INT08U	refactorStat_3;	//44�ع�״̬3
	CPU_INT08U	refactorStat_4;	//45�ع�״̬4
	CPU_INT08U	refactorStat_5;	//46�ع�״̬5
	CPU_INT08U	refactorStat_6;	//47�ع�״̬6
	CPU_INT08U	refactorStat_7;	//48�ع�״̬7
	CPU_INT08U	refactorStat_8;	//49�ع�״̬8
	CPU_INT08U	refactorStat_9;	//50�ع�״̬9
	CPU_INT08U	refactorStat_10;	//51�ع�״̬10
	CPU_INT08U	refactorStat_11;	//52�ع�״̬11
	CPU_INT08U	refactorStat_12;	//53�ع�״̬12
	CPU_INT08U	refactorStat_13;	//54�ع�״̬13
	CPU_INT08U	refactorStat_14;	//55�ع�״̬14
	CPU_INT08U	refactorStat_15;	//56�ع�״̬15
	CPU_INT08U	refactorStat_16;	//57�ع�״̬16
} SBT_DCS_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U	operatingVol;	//2������ѹU8
	CPU_INT08U	basebandCur;	//3��������U8
	CPU_INT08U	AGC;	//4����AGC
	CPU_INT08U	pwrAmplifierCur;	//5���ŵ���U8
	CPU_INT08U	pwrAmplifierTemp;	//6�����¶�U8
	CPU_INT08U	reserve[3];	//7-9����
	CPU_INT08U	workStat_UpMode;	//10���й���״̬��
	CPU_INT08U	channel_RxFirstLevel;	//11����һ������������
	CPU_INT08U	channelWork_DownModulation;	//12���е��ƿ��ص�
	CPU_INT08U	rcChannelSNR;	//13ң��ͨ��SNR
	CPU_INT08U	rcDirectCmdCnt;	//14ң��ֱ��ָ�����(����)
	CPU_INT08U	rcInjectionCmdCnt;	//15ң����עָ�����(����)
	CPU_INT08U	indirectCmdCnt_Rx;	//16���ָ����ռ�����
	CPU_INT08U	selfCheck1_RepairableCnt;	//17���������޸���ת������
	CPU_INT08U	selfCheck2_reserve1;	//18������ʹ�ܵ�
	CPU_INT08U	ddt_CannotFixErr;	//19����Flash��ȡ����TMR�޷��޸������ǵ�
	CPU_INT16U	storageAddr;	//20-21�洢��ַָ��
	CPU_INT16U	sequentialPlayback;	//22-23˳��ط�ָ��
	CPU_INT16U	curPlayback;	//24-25��ǰ�ط�ָ��
	CPU_INT08U	fixedMemory_Stat;	//26�̴�״̬��
	CPU_INT08U	loadFrameCnt;	//27�غɴ���֡���ռ���
	CPU_INT08U	telemetryFrameRxCnt;	//28ң��֡���ռ���
	CPU_INT08U	downTelemetryFrameCnt;	//29����ң��֡����
	CPU_INT08U	injectionCorrFrameCnt;	//30������ע������ȷ֡����
	CPU_INT08U	injectionErrFrameCnt;	//31������ע���ݴ���֡����
	CPU_INT08U	reserve_2;	//32-33����
	CPU_INT08U	reserve_3;	//32-33����
	CPU_INT08U	monitorFPGAVer;	//34���FPGA�汾
	CPU_INT08U	internalTestVerNum;	//35����ڲ�汾��
	CPU_INT08U	verNum;	//36����汾��
	CPU_INT08U	check;	//37У���
} XTTC_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U solid;		//�̶�ֵ
	CPU_INT08U att4Mmt[16];	//��̬��Ԫ��
	CPU_INT08U innTime[7];	//���������ڲ�ʱ��
	CPU_INT08U temp;		//�����������񴫸����¶�
	CPU_INT08U picExp;		//ͼ���ع�ֵ
	CPU_INT08U picLmt;		//ͼ����ֵ
	CPU_INT08U bkgrd;		//����ֵ
	CPU_INT08U innSign;		//EDAC���ر�־/FPGA_MRAM�Լ���/CMOS Chip ID������/ϵͳ�ڲ��������̴���
	CPU_INT08U innPara;		//ϵͳ����ģʽ/��ȡ����
	CPU_INT08U gainDb;		//��Ԫ����Чʱ��������/�����С
	CPU_INT08U dataValid;	//ʶ������/�ⲿͼ����򿪹ضϱ�ʶ/������Ч��־
	CPU_INT08U devNo;		//�ڲ�����汾��/��Ʒ�豸���
	CPU_INT08U EDACErrCnt;		//EDAC�������
	CPU_INT08U picNo[3];		//ͼ��֡��
	CPU_INT08U Att4MmtSwi;	//SAA��ֵ/SAA����ģʽ/��Ԫ���˲����ر�־λ
	CPU_INT08U starLmt;		//����Ѱ����ֵ
	CPU_INT08U followLmt;	//������ֵ
	CPU_INT08U sum;			//�ۼӺ�
} XM_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U TC_state;        //ң��ͨ��״̬
	CPU_INT08U TC_carr_devia;   //ң��ͨ���ز�Ƶƫ
	CPU_INT08U TC_SNR;          //ң��ͨ��SNR
	CPU_INT08U TC_AGC;          //ң��ͨ��AGC
	CPU_INT08U Refresh_state;   //ˢ��״̬
	CPU_INT08U sum;	            //��У��
} XKZ_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U st[499];
	CPU_INT08U sum;	            //��У��
} DLX_TM;
#pragma pack()


#endif
//GPS1_TM_TAB GPSģ����͸�ʽ1:1sһ��

#pragma pack(1)
typedef struct
{
	CPU_INT16U frame_type;          //֡����0x1001
	CPU_INT16U data_word_length;    //��Ч�����ֳ���447
	CPU_INT32U TIC_cnt;             //GPS����ʱ��(TIC����)
	CPU_INT08U gps_state;           //GPS��λ״̬
	CPU_INT08U gps_satellite_cnt;   //GPS��λ������
	CPU_INT16U GPS_week;            //GPS��
	CPU_INT32U GPS_sec;             //GPS����
	CPU_INT16U BDS_week;            //BD��:wxy�µ�ͨ��Э������
	CPU_INT32U BDS_sec;             //BD����
	CPU_INT32U UTC_sec;            //UTCʱ���ۻ���ֵ
	CPU_INT32S gps_location[3];               //GPS��λλ��X,Y,Z
	CPU_INT32S gps_V[3];              //GPS��λ�ٶ�Vx,Vy,Vz
	CPU_INT32S gps_clock_err;       //GPS�Ӳ�
	CPU_INT32S gps_clock_drift;     //GPS��Ư
	CPU_INT16U gps_PDOP;
	CPU_INT16U gps_GDOP;

	CPU_INT08U L1_chan1_satellite_num;  //L1ͨ��1���Ǻ�
	CPU_INT08U L1_chan1_state;          //L1ͨ��1״̬
	CPU_INT08U L1_chan1_SN;             //L1ͨ��1�����
	CPU_INT08U L1_chan1_ELV;            //�������̶�Ϊ0��
	CPU_INT32U L1_chan1_wj_integer;     //L1ͨ��1α����������
	CPU_INT08U L1_chan1_wj_decimal;     //L1ͨ��1α��С������
	CPU_FP32   L1_chan1_DP;             //L1ͨ��1�ز�������ֵ
	CPU_INT32S L1_chan1_cw_integer;     //L1ͨ��1�ز���λ��������
	CPU_INT16S L1_chan1_cw_decimal;     //L1ͨ��1�ز���λС������

	CPU_INT08U L1_chan2_satellite_num;
	CPU_INT08U L1_chan2_state;
	CPU_INT08U L1_chan2_SN;
	CPU_INT08U L1_chan2_ELV;
	CPU_INT32U L1_chan2_wj_integer;
	CPU_INT08U L1_chan2_wj_decimal;
	CPU_FP32   L1_chan2_DP;
	CPU_INT32S L1_chan2_cw_integer;
	CPU_INT16S L1_chan2_cw_decimal;

	CPU_INT08U L1_chan3_satellite_num;
	CPU_INT08U L1_chan3_state;
	CPU_INT08U L1_chan3_SN;
	CPU_INT08U L1_chan3_ELV;
	CPU_INT32U L1_chan3_wj_integer;
	CPU_INT08U L1_chan3_wj_decimal;
	CPU_FP32   L1_chan3_DP;
	CPU_INT32S L1_chan3_cw_integer;
	CPU_INT16S L1_chan3_cw_decimal;

	CPU_INT08U L1_chan4_satellite_num;
	CPU_INT08U L1_chan4_state;
	CPU_INT08U L1_chan4_SN;
	CPU_INT08U L1_chan4_ELV;
	CPU_INT32U L1_chan4_wj_integer;
	CPU_INT08U L1_chan4_wj_decimal;
	CPU_FP32   L1_chan4_DP;
	CPU_INT32S L1_chan4_cw_integer;
	CPU_INT16S L1_chan4_cw_decimal;

	CPU_INT08U L1_chan5_satellite_num;
	CPU_INT08U L1_chan5_state;
	CPU_INT08U L1_chan5_SN;
	CPU_INT08U L1_chan5_ELV;
	CPU_INT32U L1_chan5_wj_integer;
	CPU_INT08U L1_chan5_wj_decimal;
	CPU_FP32   L1_chan5_DP;
	CPU_INT32S L1_chan5_cw_integer;
	CPU_INT16S L1_chan5_cw_decimal;

	CPU_INT08U L1_chan6_satellite_num;
	CPU_INT08U L1_chan6_state;
	CPU_INT08U L1_chan6_SN;
	CPU_INT08U L1_chan6_ELV;
	CPU_INT32U L1_chan6_wj_integer;
	CPU_INT08U L1_chan6_wj_decimal;
	CPU_FP32   L1_chan6_DP;
	CPU_INT32S L1_chan6_cw_integer;
	CPU_INT16S L1_chan6_cw_decimal;

	CPU_INT08U L1_chan7_satellite_num;
	CPU_INT08U L1_chan7_state;
	CPU_INT08U L1_chan7_SN;
	CPU_INT08U L1_chan7_ELV;
	CPU_INT32U L1_chan7_wj_integer;
	CPU_INT08U L1_chan7_wj_decimal;
	CPU_FP32   L1_chan7_DP;
	CPU_INT32S L1_chan7_cw_integer;
	CPU_INT16S L1_chan7_cw_decimal;

	CPU_INT08U L1_chan8_satellite_num;
	CPU_INT08U L1_chan8_state;
	CPU_INT08U L1_chan8_SN;
	CPU_INT08U L1_chan8_ELV;
	CPU_INT32U L1_chan8_wj_integer;
	CPU_INT08U L1_chan8_wj_decimal;
	CPU_FP32   L1_chan8_DP;
	CPU_INT32S L1_chan8_cw_integer;
	CPU_INT16S L1_chan8_cw_decimal;

	CPU_INT08U L1_chan9_satellite_num;
	CPU_INT08U L1_chan9_state;
	CPU_INT08U L1_chan9_SN;
	CPU_INT08U L1_chan9_ELV;
	CPU_INT32U L1_chan9_wj_integer;
	CPU_INT08U L1_chan9_wj_decimal;
	CPU_FP32   L1_chan9_DP;
	CPU_INT32S L1_chan9_cw_integer;
	CPU_INT16S L1_chan9_cw_decimal;

	CPU_INT08U L1_chan10_satellite_num;
	CPU_INT08U L1_chan10_state;
	CPU_INT08U L1_chan10_SN;
	CPU_INT08U L1_chan10_ELV;
	CPU_INT32U L1_chan10_wj_integer;
	CPU_INT08U L1_chan10_wj_decimal;
	CPU_FP32   L1_chan10_DP;
	CPU_INT32S L1_chan10_cw_integer;
	CPU_INT16S L1_chan10_cw_decimal;

	CPU_INT08U B1_chan1_satellite_num;  //B1ͨ��1���Ǻ�
	CPU_INT08U B1_chan1_state;          //B1ͨ��1״̬
	CPU_INT08U B1_chan1_SN;             //B1ͨ��1�����
	CPU_INT08U B1_chan1_ELV;            //B1ͨ��1��������wxy
	CPU_INT32U B1_chan1_wj_integer;     //B1ͨ��1α����������
	CPU_INT08U B1_chan1_wj_decimal;     //B1ͨ��1α��С������
	CPU_FP32   B1_chan1_DP;             //B1ͨ��1�ز�������ֵ
	CPU_INT32S B1_chan1_cw_integer;     //B1ͨ��1�ز���λ��������
	CPU_INT16S B1_chan1_cw_decimal;     //B1ͨ��1�ز���λС������

	CPU_INT08U B1_chan2_satellite_num;
	CPU_INT08U B1_chan2_state;
	CPU_INT08U B1_chan2_SN;
	CPU_INT08U B1_chan2_ELV;
	CPU_INT32U B1_chan2_wj_integer;
	CPU_INT08U B1_chan2_wj_decimal;
	CPU_FP32   B1_chan2_DP;
	CPU_INT32S B1_chan2_cw_integer;
	CPU_INT16S B1_chan2_cw_decimal;

	CPU_INT08U B1_chan3_satellite_num;
	CPU_INT08U B1_chan3_state;
	CPU_INT08U B1_chan3_SN;
	CPU_INT08U B1_chan3_ELV;
	CPU_INT32U B1_chan3_wj_integer;
	CPU_INT08U B1_chan3_wj_decimal;
	CPU_FP32   B1_chan3_DP;
	CPU_INT32S B1_chan3_cw_integer;
	CPU_INT16S B1_chan3_cw_decimal;

	CPU_INT08U B1_chan4_satellite_num;
	CPU_INT08U B1_chan4_state;
	CPU_INT08U B1_chan4_SN;
	CPU_INT08U B1_chan4_ELV;
	CPU_INT32U B1_chan4_wj_integer;
	CPU_INT08U B1_chan4_wj_decimal;
	CPU_FP32   B1_chan4_DP;
	CPU_INT32S B1_chan4_cw_integer;
	CPU_INT16S B1_chan4_cw_decimal;

	CPU_INT08U B1_chan5_satellite_num;
	CPU_INT08U B1_chan5_state;
	CPU_INT08U B1_chan5_SN;
	CPU_INT08U B1_chan5_ELV;
	CPU_INT32U B1_chan5_wj_integer;
	CPU_INT08U B1_chan5_wj_decimal;
	CPU_FP32   B1_chan5_DP;
	CPU_INT32S B1_chan5_cw_integer;
	CPU_INT16S B1_chan5_cw_decimal;

	CPU_INT08U B1_chan6_satellite_num;
	CPU_INT08U B1_chan6_state;
	CPU_INT08U B1_chan6_SN;
	CPU_INT08U B1_chan6_ELV;
	CPU_INT32U B1_chan6_wj_integer;
	CPU_INT08U B1_chan6_wj_decimal;
	CPU_FP32   B1_chan6_DP;
	CPU_INT32S B1_chan6_cw_integer;
	CPU_INT16S B1_chan6_cw_decimal;

	CPU_INT08U B1_chan7_satellite_num;
	CPU_INT08U B1_chan7_state;
	CPU_INT08U B1_chan7_SN;
	CPU_INT08U B1_chan7_ELV;
	CPU_INT32U B1_chan7_wj_integer;
	CPU_INT08U B1_chan7_wj_decimal;
	CPU_FP32   B1_chan7_DP;
	CPU_INT32S B1_chan7_cw_integer;
	CPU_INT16S B1_chan7_cw_decimal;

	CPU_INT08U B1_chan8_satellite_num;
	CPU_INT08U B1_chan8_state;
	CPU_INT08U B1_chan8_SN;
	CPU_INT08U B1_chan8_ELV;
	CPU_INT32U B1_chan8_wj_integer;
	CPU_INT08U B1_chan8_wj_decimal;
	CPU_FP32   B1_chan8_DP;
	CPU_INT32S B1_chan8_cw_integer;
	CPU_INT16S B1_chan8_cw_decimal;

	CPU_INT08U B1_chan9_satellite_num;
	CPU_INT08U B1_chan9_state;
	CPU_INT08U B1_chan9_SN;
	CPU_INT08U B1_chan9_ELV;
	CPU_INT32U B1_chan9_wj_integer;
	CPU_INT08U B1_chan9_wj_decimal;
	CPU_FP32   B1_chan9_DP;
	CPU_INT32S B1_chan9_cw_integer;
	CPU_INT16S B1_chan9_cw_decimal;

	CPU_INT08U B1_chan10_satellite_num;
	CPU_INT08U B1_chan10_state;
	CPU_INT08U B1_chan10_SN;
	CPU_INT08U B1_chan10_ELV;
	CPU_INT32U B1_chan10_wj_integer;
	CPU_INT08U B1_chan10_wj_decimal;
	CPU_FP32   B1_chan10_DP;
	CPU_INT32S B1_chan10_cw_integer;
	CPU_INT16S B1_chan10_cw_decimal;

    CPU_INT08U curWorkModule;//444����ģ�鹤��״̬
	CPU_INT08U version;  	//445����汾��
    CPU_INT08U orbitStat;	//446����״̬-����
	CPU_INT08U reserved[64];//����
	CPU_INT08U sum;         //��У��
} GNSS_TM;
#pragma pack()


///*-------------------GNB_TM_Tab���ܰ�ң���---------------------------------*/
#pragma pack(1)
typedef struct
{
#if 1
	//CPU_INT16U syn;				//0-1ͬ����
	CPU_INT16U frameCnt;               //2-3ң��֡����
    CPU_INT16U frameLen;               //4-5֡��
    CPU_INT08U cmdCnt;                  //6ָ�����
    CPU_INT08U cmdRxStat;               //7ָ�����״̬
    CPU_INT16S sunAngle_a1;          //8-9̫���Ǽ�a1
    CPU_INT16S sunAngle_a2;          //10-11̫���Ǽ�a2
    CPU_INT16S sunAngle_a3;          //12-13̫���Ǽ�a3
    CPU_INT16S sunAngle_a4;          //14-15̫���Ǽ�a4
    CPU_INT16S sunAngle_b1;          //16-17̫���Ǽ�b1
    CPU_INT16S sunAngle_b2;          //18-19̫���Ǽ�b2
    CPU_INT16S sunAngle_b3;          //20-21̫���Ǽ�b3
    CPU_INT16S sunAngle_b4;          //22-23̫���Ǽ�b4
    CPU_INT16U curBackup_1;            //24-25��������1
    CPU_INT16U curBackup_2;            //26-27��������2
    CPU_INT16U curBackup_3;            //28-29��������3
    CPU_INT16U curBackup_4;            //30-31��������4
    CPU_INT16U curBackup_5;            //32-33��������5
    CPU_INT16U curBackup_6;            //34-35��������6
    CPU_INT16U curBackup_7;            //36-37��������7
    CPU_INT16U curBackup_8;            //38-39��������8
    CPU_INT16U maskTemp_PosX;          //40-41+X������Ĥ�¶�
    CPU_INT16U maskTemp_NegZ;          //42-43-Z������Ĥ�¶�
    CPU_INT16S satSeparated_2;         //44-45�Ǽ������ź�2
    CPU_INT16S busVoltage;           //46-47ĸ�ߵ�ѹң��
    CPU_INT16S batteryVoltage;       //48-49�������ѹң��
    CPU_INT16S loadCurrent;          //50-51���ص���ң��
    CPU_INT16S solarArrayCur;        //52-53̫�������ң��
    CPU_INT16S batteryDischargeCur;  //54-55������ŵ����ң��
    CPU_INT16S batteryChargeCur;     //56-57�����������ң��
    CPU_INT16S GNSS_RReceiver;       //58-59GNSS-R���ջ�������ѹ
    CPU_INT16S GNSS_RAmplifier;      //60-61GNSS-R����Ź����ѹ
    CPU_INT16S magnetometer_X;       //62-63��ǿ��X�ų�ǿ��
    CPU_INT16S magnetometer_Y;       //64-65��ǿ��Y�ų�ǿ��
    CPU_INT16S magnetometer_Z;       //66-67��ǿ��Z�ų�ǿ��
    CPU_INT16S FPGAMainDownload;       //68-69FPGA���������ָʾ
    CPU_INT16S FPGABackupDownload;     //70-71FPGA���������ָʾ
    CPU_INT16S DSPDownload;            //72-73DSP����������ָʾ
    CPU_INT16U reserve;                //74-75����
    CPU_INT16U batteryTemp_1;        //76-77�������¶�ң��1
    CPU_INT16U batteryTemp_2;        //78-79�������¶�ң��2
    CPU_INT16U receiverTemp;         //80-81GNSS-R���ջ��¶�
    CPU_INT16U upAntennaTemp;        //82-83�������ߵ�����¶�
    CPU_INT16U downAntennaTemp;    //84-85�������ߵ�����¶�
    CPU_INT16U downAntennaTemp_2;    //86-87�������ߵ�����¶�2
    CPU_INT16U SSATemp;           //88-89��������A�¶�
    CPU_INT16U SSBTemp;           //90-91��������B�¶�
    CPU_INT16S FPGADownload;           //92-93FPGA����������ָʾ
    CPU_INT16U volBackup_2;            //94-95��ѹ����2
    CPU_INT16U volBackup_3;            //96-97��ѹ����3
    CPU_INT16U volBackup_4;            //98-99��ѹ����4
    CPU_INT16U GSTemp;           //100-101�����¶�
    CPU_INT16U clapboardTemp;       //102-103�������¶�
    CPU_INT16U FWTriTemp;           //104-105��������¶�
    CPU_INT16U FWSTemp;           //106-107��������¶�
		/*21�������¶�*/
		CPU_INT16U groupInertiaTemp ;           
    CPU_INT16U starSensorTemp ;           
    CPU_INT16U cageTemp ;           
    CPU_INT16U coolThrust1Temp ;         
    CPU_INT16U coolThrust2Temp ;
		CPU_INT16U coolBottleTemp  ;           
    CPU_INT16U coolValveTemp  ;           
    CPU_INT16U catalyticBed1Temp  ;                      
    CPU_INT16U catalyticBed3Temp  ;  
    CPU_INT16U catalyticBed5Temp  ;         
		CPU_INT16U catalyticBed6Temp  ;           
    CPU_INT16U catalyticBed7Temp  ;          
    CPU_INT16U catalyticBed8Temp  ;                 
    CPU_INT16U fuelTankTemp ;         
    CPU_INT16U fuelLineTemp ;          
    CPU_INT16U pressureSensorTemp ;          
		CPU_INT16U solenoidValve1Temp ;           
    CPU_INT16U solenoidValve2Temp ;           
    CPU_INT16U observationTemp ;   
		CPU_INT16U ZDBTemp  ; 
    CPU_INT16U batteryTemp  ;		

		
    CPU_INT16U battTemp_1_HeatCtrl;           //108-109�����¶�1-�ȿ�
    CPU_INT16U battTemp_2_HeatCtrl;           //110-111�����¶�2-�ȿ�
    CPU_INT16U OBCTemp;           //112-113�ۺϵ����¶�
    CPU_INT16U XTTCATemp;          //114-115��Ƶ�����������¶�
    CPU_INT16U XTTCBTemp;          //116-117��Ƶ�����������¶�
    CPU_INT16U PCDUTemp;          //118-119��Դ�������¶�
    CPU_INT16U DCSTemp;          //120-121DCS�غ��¶�
    CPU_INT16U downAntennaCombinerTemp;          //122-123GNSS-R�������ߺ�·���¶�
    CPU_INT16U maskTemp_NegY;          //124-125-Y������Ĥ�¶�
    CPU_INT16U V24Power;               //126-127V24���
    CPU_INT16U V25Power;               //128-129V25���
    CPU_INT16S V11Power;               //130-131V11��磨GPS��磩
    CPU_INT16U V14Power;               //132-133V14���
    CPU_INT16S CPUAPower;              //134-135CPUA���
    CPU_INT16S CPUBPower;              //136-137CPUB���
    CPU_INT16S CPUBSignal;             //138-139CPUBȨ�ź�
    CPU_INT16S busbar_5V;            //140-141ĸ��5V
    CPU_INT16S busbar_12V;           //142-143ĸ��12V
    CPU_INT16U busbar_28V;           //144-145ĸ��28V
    CPU_INT16S Magnetorquer_Pos1;    //146-147��������1+����
    CPU_INT16S Magnetorquer_Neg1;    //148-149��������1-����
    CPU_INT16S Magnetorquer_Pos2;    //150-151��������2+����
    CPU_INT16S Magnetorquer_Neg2;    //152-153��������2-����
    CPU_INT16S Magnetorquer_Pos3;    //154-155��������3+����
    CPU_INT16S Magnetorquer_Neg3;    //156-157��������3-����
    CPU_INT16U temp;                 //158-159�ۺϵ��ӵ����¶�
    CPU_INT16U satSeparated_1;          //160�Ǽ������ź�1
		
		
    /*CPU_INT08U heater_15;               //������15����
    CPU_INT08U heater_14;               //������14����
    CPU_INT08U heater_13;               //������13����
    CPU_INT08U heater_12;               //������12����
    CPU_INT08U heater_11;               //PCDU����������
    CPU_INT08U heater_10;               //����B����������
    CPU_INT08U heater_9;                //����A����������
    CPU_INT08U heater_8_1;              //161���Ƽ���������
    CPU_INT08U heater_7;                //GNSS���ӵ���ż���������
    CPU_INT08U heater_6;                //GNSS���ӵ���ż���������
    CPU_INT08U heater_5;                //GNSS��·������������
    CPU_INT08U heater_4;                //GNSS��Ƶ���ջ�����������
    CPU_INT08U heater_3;                //�ۺϵ��Ӽ���������
    CPU_INT08U heater_2;                //���ؼ�����������
    CPU_INT08U heater_1;                //���ؼ�����������*/
    CPU_INT16U end_1;                  //162-163ң�������־AABB
    CPU_INT16U end_2;                  //164-165ң�������־CCDD
    CPU_INT08U end_3;                   //166ң�������־EE
    CPU_INT08U check;                   //167���У��
#endif
} GNB_TM;
#pragma pack()
#pragma pack(1)
typedef struct
{
	//CPU_INT32Usyn;	//0~3ͬ����0xeb90000f��ԭ��û��
	CPU_INT16U	busVoltage;	//4-5ĸ�ߵ�ѹң��
	CPU_INT16U	batteryVoltage;	//6-7���ص�ѹң��
	CPU_INT16U	solarArrayCur;	//8-9̫�������ң��
	CPU_INT16U	batteryDischargeCur;	//10-11������ŵ����ң��
	CPU_INT16U	batteryChargeCur;	//12-13�����������ң��
	CPU_INT16U	loadCurrent;	//14-15���ص���ң��
	CPU_INT16U	voltage_5_2V;	//16-175.2V��ѹң��
	CPU_INT16U	voltage_12V;	//18-1912V��ѹң��
	CPU_INT16U	voltage_neg12V;	//20-21-12V��ѹң��
	CPU_INT16U	voltage_6V;	//22-236V��ѹң��
	CPU_INT16U	solarArrayTemp_PosX;	//24-25+X�����¶�
	CPU_INT16U	solarArrayTemp_NegX;	//26-27-X�����¶�
	CPU_INT16U	temperature_1;	//28-29PCDU�¶�
	CPU_INT16U	reserve;	//30-31Ԥ��
	CPU_INT08U	dischargeSwStat;	//32�ŵ翪��״̬��
	CPU_INT08U	GSAPwrStat;	//33����A����״̬��
	CPU_INT08U	EP3PwrStat;	//34���ƿ������3����״̬��
	CPU_INT08U	receiverPwrStat_Neg12V;	//35���ջ�-12V����״̬��
	CPU_INT08U	hotKnifeFlightPlugStat_2;	//36�ȵ����в�ͷ2״̬��
	CPU_INT16U	currentBattery;	//37-38��ǰ����
	CPU_INT08U	instructionexecutionCnt;	//39ָ��ִ�м���
	CPU_INT08U	warmResetCnt;	//40�ȸ�λ����
	CPU_INT08U	fullAmpereHourVol;	//41��ע����������ʱ����(���ص�ѹ)
	CPU_INT08U	fullAmpereHourCur;	//42��ע����������ʱ����(���ص���)
	CPU_INT08U	internalVer;	//43����ڲ��汾
	CPU_INT08U	lowerCpuStat;	//44��λ������״̬
	CPU_INT08U	reserve_2[18];	//45-62Ԥ��
	CPU_INT08U	check;	//63У���

} PCDU_TM;
#pragma pack()

//-------------------------AOCC�ڽṹ��
// ����ң�����ݰ�(16 Byte)
#pragma pack(1)
typedef struct
{
	unsigned char GuSw; 				// ��������״̬��
	short HeaderDriftX; 				// ����X���ͷƯ��
	short HeaderDriftY; 				// ����Y���ͷƯ��
	short HeaderDriftZ; 				// ����Z���ͷƯ��
	unsigned char GuSpdX[3];			// ����X����ٶ�
	unsigned char GuSpdY[3];			// ����Y����ٶ�
	unsigned char GuSpdZ[3];			// ����Z����ٶ�
	short GuTempX;
	short GuTempY;
	short GuTempZ;
	short bk;
}AOCC_GURM;
#pragma pack()

// ����ң�����ݰ�(36 Byte)
#pragma pack(1)
typedef struct
{
	float StQsi[4];					    // ST��̬��Ԫ��
	unsigned int StTimeStampS;			// ST�ڲ�ʱ����
	short StSpd[3];						// STα����
	unsigned char StTimeStampMS[3];		// ST�ڲ�ʱ�����
	char StCCDTemp;						// STCCD�¶�
	char StRateQuality;					// ST��������
	char StWkSwA;						// ST�����С/ST1��Ԫ����Чʱ��������
	char StWkSwB;						// ST����������Ч��ʶ/ST1�ⲿͼ�����ʶ/ST1ʶ������
	char StWkSwC;						// ST��Ԫ���˲�����/ST1SAA����ģʽ/ST1SAA��ֵ
	char StBgValue;						// ST����ֵ
	char bk;					
	short StDiffTime;					// �����ع�ʱ���ֵ
    short StSunAng;
    short StEthAng;
    short bk2;

}AOCC_STRM;
#pragma pack()

// ��ǿ��ң�����ݰ�(6 Byte)
#pragma pack(1)
typedef struct
{
	short MagMm[3];						// ��ǿ�Ʋ���ֵ
	short bk;
}AOCC_MMRM;
#pragma pack()


///*-------------------AOCC_TM_Tab�˹��ң���---------------------------------*/
//#pragma pack(1)
//typedef struct
//{
//		CPU_INT08U syn[2]; 				
//		CPU_INT16U TM_frame_cnt;			// �˹��ң��֡����

//		// ��̬ң��
//		unsigned char AttCtlDiffX[3];		// X����̬�ǿ���ƫ��
//		unsigned char AttCtlDiffY[3];		// Y����̬�ǿ���ƫ��
//		unsigned char AttCtlDiffZ[3];		// Z����̬�ǿ���ƫ��
//		unsigned char AttCtlGlobalDiffX[3]; // X����̬��ȫ�ֿ���ƫ��
//		unsigned char AttCtlGlobalDiffY[3]; // Y����̬��ȫ�ֿ���ƫ��
//		unsigned char AttCtlGlobalDiffZ[3]; // Z����̬��ȫ�ֿ���ƫ��
//		unsigned char UVecSatSunBX[3];		// X�᱾��ϵ������ָ��̫���ĵ�λʸ��
//		unsigned char UVecSatSunBY[3];		// Y�᱾��ϵ������ָ��̫���ĵ�λʸ��
//		unsigned char UVecSatSunBZ[3];		// Z�᱾��ϵ������ָ��̫���ĵ�λʸ��
//		unsigned char IneSpdX[3];			// X����Խ��ٶ�
//		unsigned char IneSpdY[3];			// Y����Խ��ٶ�
//		unsigned char IneSpdZ[3];			// Z����Խ��ٶ�
//		short SpdCtlDiffX;					// X����ٶȿ���ƫ��
//		short SpdCtlDiffY;					// Y����ٶȿ���ƫ��
//		short SpdCtlDiffZ;					// Z����ٶȿ���ƫ��
//		short SpdCtlGlbDiffX;				// X��ȫ�ֽ��ٶȿ���ƫ��
//		short SpdCtlGlbDiffY;				// Y��ȫ�ֽ��ٶȿ���ƫ��
//		short SpdCtlGlbDiffZ;				// Z��ȫ�ֽ��ٶȿ���ƫ��
//		unsigned short SailSunAng;			// ����̫����
//		short cosASInteSum; 				// ̫�������һ���
//		short MagTbl[3];					// �ų�������
//		short MagInUse[3];					// ʵ��ʹ�õĴų�����  
//		float AttCfmQuat[4];				// ����ϵ��Թ���ϵ����Ԫ��
//		float AssAngf[3];					// ģ��̫���Ǽ�̫��ʸ��
//		float ObtPhsAng;					// ����Ƕ������ǹ����λ��
//		float ObtPhsAngStr; 				// ����Ƕ��յ������ǹ����λ��
//		float ObtSunAng;					// ����Ƕ��չ��̫����
//		float DiffQuat[3];					// �����Ԫ��
//		float Qir[4];						// �滮��Ԫ��
//		short Wrr[3];						// �滮���ٶ�
//		short AssAngM[3]; 					// ģ��̫���Ǽ������̫����
//		short AssAngQ[3];					// ��̬�������̫����
//		short bkt21;
//		float QIEnd[4];						// Ŀ����Ԫ��
//		float AngObt[3];					// ���ϵ��̬��
//		
//		// ����ң��
//		short StarAngMmt[3];				// ���ǽǶ���
//		short FeedForwardMmt[3];			// ǰ������
//		short PIDCtrlMmt[3];				// PID��������
//		short RwSpdCmd[4];					// ����ָ��ת��
//		short MtCmdMmt[3];					// ����������ָ��ž�
//		short Tdw[3];						// ��̬�����Ǽ��ٶ�ǰ������ 	
//		short Tmmt[3];						// ���Ƕ�����������
//		short Tquat[3]; 					// �����Ԫ����������
//		short Tq0inte[3];					// �����Ԫ���겿���ַ�������
//		short StarAngMmtB[3];				// ����ϵ�����ǽǶ���
//		short bkc0;

//		// ����ң��
//        unsigned char MoveSdyState; 			// ������̬״̬
//        unsigned char CmdLockStatus;			// ָ����״̬
//        unsigned char TaskRTPlanStatus;				// ����ʵʱ����״̬
//        unsigned char TaskWaitNum;				// �˿�ָ���������
//        unsigned char WkStartTime[4];			// ��ǰ������ʼʱ��
//        unsigned char WkEndTime[4];				// ��ǰ��������ʱ��
//        unsigned char NextWkStaTime[4];			// ��һ��ָ�ʼʱ��
//        unsigned char NextWkEndTime[4];			// ��һ��ָ�����ʱ��
//        unsigned char TaskHtRejectTime[4];		// ��ʷ����ʱ��
//        unsigned short TaskUploadNum;			// �˿�ָ����ע����
//        unsigned short TaskInvalidNum;			// �˿�ָ����Ч����
//        unsigned short TaskPushNum;				// �˿�ָ����Ӽ���
//        unsigned short TaskRejectNum;			// �˿�ָ����ռ���
//        unsigned short TaskExcuteNum;			// �˿�ָ��ִ�м���
//        unsigned short TaskClearNum;			// �˿�ָ���������
//        unsigned short TaskGeneNum;				// �˿�ָ���������ɼ���
//		short bke2;
//        unsigned char TaskHtReject;				// ��ʷ����״̬
//        unsigned char  TaskAcceptStatus;		// ��������ձ�־
//		unsigned short CurrentCmdId;			// ��ǰָ���
//		unsigned char CurrentCordSys[2];		// ��ǰ����ϵ
//		unsigned char staMode[2]; 				// ��ǰ������ʼ����ϵ
//		unsigned short TargetInfoNum;			// ��������Ŀ
//		short MoveEulAngSin;					// ��Ԫ������ŷ�����sin������ֵ
//		unsigned short PlQtClimbTime;			// �滮��Ԫ��������ʱ��t1
//		unsigned short PlQtConstTime;			// �滮��Ԫ�����ٶ�ʱ��t2
//		unsigned short PlSpSlowTime;			// �滮���ٶȼ��ٶ�ʱ��t4
//		unsigned short PlSpAccTime; 			// �滮���ٶȼ��ٶ�ʱ��t5
//		short BaseSpdPlan[3];					// ��׼�滮���ٶ�
//		short MoveSpdPlan[3];					// �����滮���ٶ�
//		short MoveEulerAxis[3]; 				// ��Ԫ�������滮ŷ����
//		short MoveEulerAng; 					// ��Ԫ�������滮ŷ����� 
//		unsigned short MoveEulerSpdMax; 		// �滮ŷ����������ٶ�
//		unsigned short MoveTimePlan;			// �滮������ʱ��

//	
//		// �������ң��
//		unsigned char ObtLockStatus;			// ָ����״̬
//		unsigned char ObtTaskWaitNum;			// ���ָ���������
//		unsigned short ObtTaskUploadNum;		// ���ָ����ע����
//		unsigned short ObtTaskInvalidNum;		// ���ָ����Ч����
//		unsigned short ObtTaskManuNum;			// ���ָ��滮����
//		unsigned short ObtTaskPushNum;			// ���ָ����Ӽ���
//		unsigned short ObtTaskRejectNum;		// ���ָ����ռ���
//		unsigned short ObtTaskClearNum; 		// ���ָ���������
//		char ObtRejectStatus;					// ���ָ�����״̬
//		unsigned char ThrustEnable; 			// �ƽ�ʹ�ܱ�־
//		unsigned char JetStaTime[4];			// ����ָ�ʼʱ��
//		unsigned char JetEndTime[4];			// ����ָ�����ʱ��
//		unsigned char JetNextStaTime[4];		// ��һ������ָ�ʼʱ��
//		unsigned char JetNextEndTime[4];		// ��һ������ָ�����ʱ��
//		unsigned short JetPushNum;				// ����������Ӽ���
//		unsigned char JetWaitNum;				// �������д�������
//		unsigned short JetExcuteNum;			// ��������ִ�м���
//		unsigned short JetClearNum; 			// ���������������
//		unsigned char ObtCtrlEnble; 			// ���ʹ�ܿ���
//		unsigned char ObtCtrlInProcess; 		// ��ؽ����б�־
//		char bkc2[3];

//		// ���ң��
//		float SemiMajorAxis;				// ����볤��
//		float EccenRatio;					// ���ƫ����
//		float DeclAngle;					// ���������
//		float LatiArgu; 					// ���γ�ȷ���
//		float AscenNode;					// ���������ྭ
//		float PerigeeArgu;					// ������ص����
//		float MA;							// ���ƽ�����
//		float FA;							// ���������
//		float ObtSpd;						// ������ٶ�
//		float SrcSemiMajorAxis; 			// ��ʼ������ư볤��
//		float SrcEccenRatio;				// ��ʼ�������ƫ����
//		float SrcDeclAngle; 				// ��ʼ������ƹ�����
//		float SrcLatiArgu;					// ��ʼ�������������ྭ
//		float SrcAscenNode; 				// ��ʼ������ƽ��ص����
//		float SrcPerigeeArgu;				// ��ʼ�������γ�ȷ���
//		float SrcMA;						// ��ʼ�������ƽ�����
//		float GPSWgsPos[3]; 				// WGSλ��
//		float GPSWgsSpd[3]; 				// WGS�ٶ�
//		unsigned char SrcRefClk[4];			// ��ʼ�����Ԫ
//		unsigned char ObtCalTime[4];		// �������ʱ��
//		unsigned char ObtCalUw; 			// ������ݽ���״̬
//		unsigned char ObtDtStatus;			// �������״̬
//		unsigned char ObtUwSelect;			// ������עʹ�ù������
//		unsigned char ObtDtType;				// ������ע��������

//		// ģʽ״̬ ң��
//		unsigned short AocsStatus;				// AOCS״̬��
//		unsigned short SysModSet;			// ϵͳ����ģʽ
//		unsigned char SunshineFlag; 			// ����̬����Ĺ�������־
//		unsigned char SysWkMod; 				// ����ģʽ
//		unsigned char Breakaway;				// �Ǽ������ź�
//		unsigned char StSelection;				// ��������ѡ���־
//		unsigned char GuSelection;				// ����ѡ���־
//		unsigned char AttInPlaceFlag;			// ��̬/������־
//		unsigned char RcPkgId;					// ң�ذ�ID����
//		unsigned char RcPkgErrId;				// ң�ذ����ݼ��
//		unsigned short RcPkgCnt;				// ң�ذ���ע����
//		unsigned char SunCptr;					// ����̫����־
//		unsigned char AocsRunTime;				// ���ʵ������ʱ��
//		unsigned char timeJcCnt;				// �۵�ʱ���������
//		unsigned char memExceptCnt; 			// MARM3ȡ2�쳣����
//		unsigned char MagUseMod;				// �ų����㷽ʽ
//		unsigned char RwSelection;				// �������ѡ��
//		unsigned short memExceptAddr;			// MRAM�ڴ��쳣��ַƫ��
//		unsigned short MagDiffCnt;				// ��ǿ�Ʋ�һ���Լ���
//		unsigned char SunshineFlagM;			// ģ̫�õ��Ĺ�������־
//		unsigned char SimRw;					// ʹ��ģ�����
//		unsigned char SimMt;					// ʹ��ģ��Ű�
//		unsigned char DlxOn;					// ����ѧģʽ��־
//		unsigned char AttBase;					// ��̬��׼
//		unsigned char SpdBase;					// ���ٶȻ�׼
//		unsigned char ForceMoveFlag;			// ǿ����־
//		unsigned char PwrLowNum;				// ��ԴΣ������
//		unsigned short PwrLowStatus;			// ��ԴΣ����־
//		unsigned char SimSt;					// ģ������
//		unsigned char SimGu;					// ģ������
//		unsigned char DlxAddOn; 				// ����ѧ�ۼ�ģʽ
//		unsigned char SunAngleMode; 			// ̫���Ǽ��㷽ʽ
//		unsigned char GuInteOn; 				// ���ݻ��ֿ���
//		unsigned char RwSelSet; 				// ϵͳ���÷������
//		unsigned char SimAss;					// ģ��̫��
//		unsigned char SimMag;					// ģ���ǿ��
//		unsigned char PpFtMonOn;				// ���ƹ��ϼ��ʹ��
//		unsigned char PpFtHandOn;				// ���ƹ��ϴ���ʹ��
//		int MagDampHealthCnt;					// �����ὡ��ģʽ����
//		int SailSunCnt; 						// ����̫���ǳ������
//		int SafeMdHealthCnt;					// ��ȫģʽ��������
//		int SrvModHealthCnt;					// ҵ��ģʽ��������
//		unsigned char DlxInsideOn;				// �ڱջ���־ʹ��
//		unsigned char DlxFastRmOn;				// ����ѧ����ң��ʹ��
//		unsigned char StSelSet; 				// ������עʹ���������
//		unsigned char StKeep;					// ������ע��������
//		unsigned short DftCordSet;				// ������ע��������ϵ
//		unsigned char MagSelSet;				// ������עʹ�ôų���׼
//		unsigned char GuSelSet; 				// ������עʹ�����ݻ�׼
//		unsigned char GuKeep;					// ������ע���ݱ���
//		unsigned char TaskLockEnble;			// ����������ʹ��
//		unsigned char StAccept; 				// ����������ǿ�ƽ���
//		unsigned char StOnoff;					// �������ñ�־
//		unsigned char SimGps;					// ģ��GPS
//		unsigned short DlxPkgNum;				// ����ѧ�հ�����
//		unsigned char ObdhRmNC;                 // �������ݰ����ݲ����־
//   	    unsigned short RwEmergyRemainCnt;		// ���ֽ��������д����
//		unsigned short DelAttUpateCnt;			// ������̬�����¼���
//		unsigned short GuIntCnt;                // ���ݻ���ʹ�ü���
//		unsigned char TimeMatain;				// ʱ��ά������
//		unsigned char bk1;

//		// ̫���Ǽ�
//		short AssCura[4];					// ģ��̫���Ǽ�a1-4����
//		short AssCurb[4];					// ģ��̫���Ǽ�b1-4����

//		// ����ң��
//        short RwSpd[4];						// ����ת��
//        unsigned short RwCur[4];			// ���ֵ���
//		short RwRealSpd[4]; 				// ����ת�ټ�¼

//		// ����ң��
//		unsigned char PpPowerOnNum; 			// ���������ϵ����
//		unsigned char PpCtrlOnNum;				// ���������ϵ����
//		unsigned char PpGasEnbleNum;			// ������ƿ���ϼ��ʹ�ܼ���
//		unsigned char PpPressEnbleNum;			// ����ѹ�����ϼ��ʹ�ܼ���
//		unsigned char PpBangEnbleNum;			// ����Bang���ϼ��ʹ�ܼ���
//		unsigned char PpAutoNum;				// �����������г������
//		unsigned char PpElectOffNum;			// �������ƽ��ػ�����
//		unsigned char PpPowerOffNum;			// �������ʶϵ����
//		unsigned char PpCtrlOffNum; 			// �������ƶϵ����
//		unsigned char PpFaultStatus;			// ������ʷ����״̬
//		unsigned char PpWkMod;					// ����ʵʱ����״̬
//		char bk;
//		unsigned int PpFaultTime;				// ������ʷ����ʱ��


//		// ��������ң��
//		short MtCmdCur[3];					// ��������ָ�����
//		short MtReColCur[3];				// �����������ɵ�����X��

//		// ����ң��
//		unsigned char GuFt[MAXGUNUM];		// ��е����/�������ݹ���״̬
//		unsigned char GuWkMod[MAXGUNUM];	// ��е����/�������ݵ�������״̬
//		unsigned char GuFtCnt[MAXGUNUM];	// ���ݹ��ϴ���
//		unsigned char GuOnOffNum[MAXGUNUM];	// ���ݿ��ػ�����
//		unsigned char StFt[MAXSTNUM];		// ��������״̬
//		unsigned char StWkMod[MAXSTNUM];	// ������������״̬
//		unsigned char StFtNum[MAXSTNUM];	// �������ϴ���
//		unsigned char StOnOffNum[MAXSTNUM];	// �������ػ�����	
//		unsigned char RwFt[4];				// ���ֹ���״̬
//		unsigned char RwWkMod[4];			// ���ֵ�������״̬
//		unsigned char RwFtNum[4];			// ���ֹ��ϴ���
//		unsigned char RwOnOffNum[4];		// ���ֿ��ػ�����
//		unsigned char MmFt[MAXMMNUM];		// ��ǿ�ƹ���״̬
//		unsigned char MmWkMod[MAXMMNUM];	// ��ǿ�Ƶ�������״̬
//		unsigned char MmFtNum[MAXMMNUM];	// ��ǿ�ƹ��ϴ���
//		unsigned char MmOnOffNum[MAXMMNUM];	// ��ǿ�ƿ��ػ�����
//		unsigned char GpsWkMod;				// GPS�����������
//		unsigned char GuFtDiag[MAXGUNUM];  	// ���ݴ�����Ͽ���
//		unsigned char StFtDiag[MAXSTNUM];	// ����������Ͽ���
//		unsigned char RwFtDiag[4];			// ���ֹ�����Ͽ���
//		unsigned char MmFtDiag[MAXMMNUM];	// ��ǿ�ƹ�����Ͽ���
//		unsigned char GpsFtDiag;			// GPS������Ͽ���	
//		unsigned char ObtDtDiag;			// ������ע������ݹ�����Ͽ���
//		unsigned char bkf9[3];
//		unsigned char GuLastErrMod[MAXGUNUM];// �����������״̬
//		unsigned char StLastErrMod[MAXSTNUM];// �����������״̬
//		unsigned char RwLastErrMod[4];		// �����������״̬
//		unsigned char MmLastErrMod[MAXMMNUM];// ��ǿ���������״̬
//		unsigned char GpsLastErrMod;		// GPS�������״̬
//		unsigned char ObtDtWkMod;			// ������ע������ݴ���״̬
//		unsigned int StLastErrTime[MAXSTNUM];// �����������ʱ��
//		unsigned int GuLastErrTime[MAXGUNUM];// �����������ʱ��
//		unsigned int MmLastErrTime[MAXMMNUM];// ��ǿ���������ʱ��
//		unsigned int RwLastErrTime[4];		// �����������ʱ��
//		unsigned int GpsLastErrTime;		// GPS�������ʱ��
//		short GuHealthScore[MAXGUNUM];		// ���ݽ�������
//		short StHealthScore[MAXSTNUM];		// ������������
//		short RwHealthScore[4];				// ���ֽ�������
//		short MmHealthScore[MAXMMNUM];		// ��ǿ�ƽ�������  

//		// ���ݡ���������ǿ��ң��
//		AOCC_GURM	RmGu[MAXGUNUM];
//		AOCC_STRM	RmSt[MAXSTNUM];
//		AOCC_MMRM	RmMag[MAXMMNUM];

//		CPU_INT08U  sum;

//} AOCC_TM;

//#pragma pack()




////����������ݽӿ�
//#pragma pack(1)
//typedef struct					//����ҵ��ע��ң�ذ�???����������ݽӿ�
//{
//	CPU_INT08U syn[2];     	//ͬ����
//	//gps
//	CPU_INT16U GpsValid;	//GPS������־
//	unsigned int Time;		//����ʱ���ۻ���
//	int WgsPos[3];			//GPS��λλ��
//	int WgsSpd[3];			//GPS��λ�ٶ�
//	unsigned char Sw;		//GPS״̬��

//	CPU_INT16U src_alarm;           //��ԴΣ����־
//	//����
//	unsigned char PwFbSt[MAXSTNUM];    //������ѹ
//	unsigned char PwFbGu[MAXGUNUM];    //���ݵ�ѹ
//	unsigned char PwFbRw[4];    		//���ֵ�ѹ
//	unsigned char PwFbMm[MAXMMNUM];    //��ǿ�Ƶ�ѹ
//	unsigned short PwFbLv;      		//��������ѹ      
//	unsigned short PwFbMt;      		//����������ѹ
//	unsigned short PwFbPp;				//��������ѹ
//	//������ң��
//	unsigned char StRmBuf[MAXSTNUM][STDATALEN];  //����ң������
//	unsigned char GuRmBuf[MAXGUNUM][MAXGUDATALEN];  //����ң������
//	unsigned char RwXRmBuf[RWDATALEN];          	//X�������ң������
//	unsigned char RwYRmBuf[RWDATALEN];          	//Y�������ң������
//	unsigned char RwZRmBuf[RWDATALEN];          	//Z�������ң������
//	unsigned char RwCRmBuf[RWDATALEN];				//бװ���ֵ�ң������
//	short MmV[MAXMMNUM][3];                      	//��ǿ�Ʋ���ֵ
//	unsigned char PPRMBuf[PPDATALEN];				//PPCUң��           
//	short AssCrnt[2][4];                        	//ģ̫����
//	unsigned char MmValid[MAXMMNUM];				//��ǿ����Ч��־
//	short MtC[3];                      				//�����������
//	unsigned char CamType;							//�������
//	unsigned char CamDataValid;						//���������Ч��
//	short CamPnt[2];								//�����Ԫ���к�
//	unsigned char bkBuf[506];
//	CPU_INT08U sum;

//} AOCC_DATA;
#pragma pack()
#pragma pack(1)
typedef struct
{
	CPU_INT08U	mode;	//ʶ����
	CPU_INT08U	xOut[3];	//X����ٶ�
	CPU_INT08U	yOut[3];	//Y����ٶ�
	CPU_INT08U	zOut[3];	//Z����ٶ�
	CPU_INT08U	fixed;		//�̶�ֵ
	CPU_INT08U	xTemp[2];	//X�������¶��ź�
	CPU_INT08U	yTemp[2];	//Y�������¶��ź�
	CPU_INT08U	zTemp[2];	//Z�������¶��ź�
	CPU_INT08U	crc;	//У���
} TL_TM;
#pragma pack()
// �˿ع㲥���ݰ�
#pragma pack(1)
typedef struct
{
	CPU_INT16U Syn;
	CPU_INT08U Mod;
	CPU_INT08U Len;
	CPU_INT08U AttTime[6];			// ��̬ʱ��
	CPU_INT16U tmp;
	CPU_INT64U ObtTime;				// ���ʱ��
	CPU_INT32U AttValid;            // ��̬��Ч��
	CPU_INT32S ObtAng[3];			// ���ϵ123ת��Ƕ�
	CPU_INT32S ObtW[3];					// ����ϵ��Թ������ϵ123ת��ŷ���Ǳ仯��
	CPU_INT32S Wib[3];				// ���Խ��ٶ��ڱ���ϵX�����
	//int Qib[4];					// ����ϵ��̬��Ԫ��
	CPU_INT32U ObtValid;			// ���������Ч��
	CPU_INT32S a;					// ����볤��
	CPU_INT32S e;					// ���ƫ����
	CPU_INT32S i;					// ������
	CPU_INT32S omg;					// ������ྭ
	CPU_INT32S w;					// ���ص����
	CPU_INT32S M;					// ƽ�����
	CPU_INT32S u;					// γ�ȷ���
	CPU_INT32S f;					// ������
	CPU_INT32S orbit_w;				// ������ٶȣ�������
	//short SunAngle;				// ̫���߶Ƚ�
	//short SatLati;
	//short SatLong;
	//unsigned int H;					// ���ľ�
	//unsigned short V;				// �����ٶ�V
	//int J2000Pos[3];				// J2000����λ��
	//int J2000Spd[3];				// J2000�����ٶ�
	//int WGS84Pos[3];				// WGS84����λ��
	//int WGS84Spd[3];				// WGS84�����ٶ�
	CPU_INT08U sum;
} T_AocsBroadcast;
#pragma pack()
void trans_cmd_send(CPU_INT08U trans_addr, CPU_INT16U tranlen,CPU_INT08U * transCmdBuf,CPU_INT08U mode);
void RS422_send(enum RS422_MACHINE name, CPU_INT08U * sendBuf,CPU_INT16U sendlen);
void RS422_rec(enum RS422_MACHINE name, CPU_INT08U * recBuf,CPU_INT16U reclen);
//void RS422_rec2(enum RS422_MACHINE2 name, CPU_INT08U * recBuf,CPU_INT16U reclen);
void RS422_send2(enum RS422_MACHINE2 name, CPU_INT08U * sendBuf,CPU_INT16U sendlen);
void AOCC_data_refresh(void);
void TM_colle_send(void);
void TM_colle_rec(void);


//extern CPU_INT08U	  CRC08CheckW( CPU_INT08U * pdata, CPU_INT32U len);
CPU_INT08U simSum(CPU_INT08U *Val, CPU_INT16U Len);
CPU_INT16U simSum16(CPU_INT08U *Val, CPU_INT16U Len);
CPU_INT08U XorSum(CPU_INT08U * p_Data, CPU_INT16U len);
CPU_INT16U XorSum16(CPU_INT16U * p_Data, CPU_INT16U len);





