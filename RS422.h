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

//发送复位基地址
#define	GNSS_SRST_BASE	0x649FE0C0	//GNSS串口
#define	XMA_SRST_BASE	0x649FE0C4	//星敏1
#define	ZH3_SRST_BASE	0x649FE0C8	//载荷3
#define	XMB_SRST_BASE	0x649FE0CC	//星敏2
#define	ZH2_SRST_BASE	0x649FE0D0	//载荷2 GNSS_R
#define	TLA_SRST_BASE	0x649FE0D4	//陀螺A
#define	TLB_SRST_BASE	0x649FE0D8	//陀螺B
#define	FLX_SRST_BASE	0x649FE0DC	//飞轮1
#define	FLY_SRST_BASE	0x649FE0E0	//飞轮2
#define	FLZ_SRST_BASE	0x649FE0E4	//飞轮3
#define	PCDU_SRST_BASE	0x649FE0E8	//PCDU
#define	EP_SRST_BASE	0x649FE0EC	//电推
#define	FLS_SRST_BASE	0x649FE0F0	//单轴飞轮
#define	SJTX_SRST_BASE	0x649FE0F4	//双机通信
#define	DLX_SRST_BASE	0x649FE0F8	//动力学
#define	GNB_SRST_BASE	0x649FE0FC	//功能板
#define	ZH1_SRST_BASE	0x649FE8C4  //载荷1 SBT_DCS
#define	CKYK_SRST_BASE	0x649FE8CC	//测控遥控



//发送缓存基地址
#define	GNSS_SCACHE_BASE	0x64800000	//GNSS串口
#define	XMA_SCACHE_BASE		0x64800400	//星敏1
#define	ZH3_SCACHE_BASE		0x64800800	//载荷3
#define	XMB_SCACHE_BASE		0x64800C00	//星敏2
#define	ZH2_SCACHE_BASE		0x64801000	//载荷2
#define	TLA_SCACHE_BASE		0x64801400	//陀螺A
#define	TLB_SCACHE_BASE		0x64801800	//陀螺B
#define	FLX_SCACHE_BASE		0x64801C00	//飞轮1
#define	FLY_SCACHE_BASE		0x64802000	//飞轮2
#define	FLZ_SCACHE_BASE		0x64802400	//飞轮3
#define	PCDU_SCACHE_BASE	0x64802800	//PCDU
#define	EP_SCACHE_BASE		0x64802C00	//电推
#define	FLS_SCACHE_BASE		0x64803000	//单轴飞轮
#define	SJTX_SCACHE_BASE	0x64803400	//双机通信
#define	DLX_SCACHE_BASE		0x64803800	//动力学
#define	GNB_SCACHE_BASE		0x64803C00	//功能板
#define	ZH1_SCACHE_BASE		0x64814400  //载荷1
#define	CKYK_SCACHE_BASE	0x64814C00	//测控遥控


//发送长度基地址（启动）
#define	GNSS_SLEN_BASE	0x649FE000	//GNSS串口
#define	XMA_SLEN_BASE	0x649FE004	//星敏1
#define	ZH3_SLEN_BASE	0x649FE008	//载荷3
#define	XMB_SLEN_BASE	0x649FE00C	//星敏2
#define	ZH2_SLEN_BASE	0x649FE010	//载荷2
#define	TLA_SLEN_BASE	0x649FE014	//陀螺A
#define	TLB_SLEN_BASE	0x649FE018	//陀螺B
#define	FLX_SLEN_BASE	0x649FE01C	//飞轮1
#define	FLY_SLEN_BASE	0x649FE020	//飞轮2
#define	FLZ_SLEN_BASE	0x649FE024	//飞轮3
#define	PCDU_SLEN_BASE	0x649FE028	//PCDU
#define	EP_SLEN_BASE	0x649FE02C	//电推
#define	FLS_SLEN_BASE	0x649FE030	//单轴飞轮
#define	SJTX_SLEN_BASE	0x649FE034	//双机通信
#define	DLX_SLEN_BASE	0x649FE038	//动力学
#define	GNB_SLEN_BASE	0x649FE03C	//功能板
#define	ZH1_SLEN_BASE	0x649FE804  //载荷1
#define	CKYK_SLEN_BASE	0x649FE80C	//测控遥控


//接收缓存基地址
#define	GNSS_RCACHE_BASE	0x64804000	//GNSS串口
#define	XMA_RCACHE_BASE		0x64805000	//星敏1
#define	ZH3_RCACHE_BASE		0x64806000	//载荷3
#define	XMB_RCACHE_BASE		0x64807000	//星敏2
#define	ZH2_RCACHE_BASE		0x64809000	//载荷2
#define	TLA_RCACHE_BASE		0x64809400	//陀螺A
#define	TLB_RCACHE_BASE		0x64809800	//陀螺B
#define	FLX_RCACHE_BASE		0x64809C00	//飞轮1
#define	FLY_RCACHE_BASE		0x6480A000	//飞轮2
#define	FLZ_RCACHE_BASE		0x6480A400	//飞轮3
#define	PCDU_RCACHE_BASE	0x6480A800	//PCDU
#define	EP_RCACHE_BASE		0x6480AC00	//电推
#define	FLS_RCACHE_BASE		0x6480B000	//单轴飞轮
#define	SJTX_RCACHE_BASE	0x6480B400	//双机通信
#define	DLX_RCACHE_BASE		0x6480B800	//动力学
#define	GNB_RCACHE_BASE		0x6480BC00	//功能板
#define	ZH1_RCACHE_BASE		0x6480D000  //载荷1
#define	CKAZGBC_RCACHE_BASE	0x6480E000	//测控A在轨编程
#define	CKBZGBC_RCACHE_BASE	0x6480F000	//测控B在轨编程
#define	CKBYC_RCACHE_BASE	0x64811000	//测控B遥测
#define	CKAYC_RCACHE_BASE	0x64811400	//测控A遥测


//接收个数基地址（复位）
#define	GNSS_RLEN_BASE		0x649FE400	//GNSS串口
#define	XMA_RLEN_BASE		0x649FE404	//星敏1
#define	ZH3_RLEN_BASE		0x649FE408	//载荷3
#define	XMB_RLEN_BASE		0x649FE40C	//星敏2
#define	ZH2_RLEN_BASE		0x649FE410	//载荷2
#define	TLA_RLEN_BASE		0x649FE414	//陀螺A
#define	TLB_RLEN_BASE		0x649FE418	//陀螺B
#define	FLX_RLEN_BASE		0x649FE41C	//飞轮1
#define	FLY_RLEN_BASE		0x649FE420	//飞轮2
#define	FLZ_RLEN_BASE		0x649FE424	//飞轮3
#define	PCDU_RLEN_BASE		0x649FE428	//PCDU
#define	EP_RLEN_BASE		0x649FE42C	//电推
#define	FLS_RLEN_BASE		0x649FE430	//单轴飞轮
#define	SJTX_RLEN_BASE		0x649FE434	//双机通信
#define	DLX_RLEN_BASE		0x649FE438	//动力学
#define	GNB_RLEN_BASE		0x649FE43C	//功能板
#define	ZH1_RLEN_BASE		0x649FEC04  //载荷1
#define	CKAZGBC_RLEN_BASE	0x649FEC08	//测控A在轨编程
#define	CKBZGBC_RLEN_BASE	0x649FEC0C	//测控B在轨编程
#define	CKBYC_RLEN_BASE		0x649FEC10	//测控B遥测
#define	CKAYC_RLEN_BASE		0x649FEC14	//测控A遥测

#define DLX_422_BASE	0x60084000       //DLX

#define OFFSET_ADDR     0x2000

/*-----------------单机地址表定义----------------------*/
#define INDIR_ADDR					0x00     //间接指令

#define PROCESS_BOARD_ADDR			0x61     //处理器板
#define ZK_MODULE_ADDR				0x62     //姿轨控模块
#define	PCDU_ADDR					0x63	//电源控制器
#define GNB_ADDR					0x64     //功能板
#define GPS_ADDR					0x65     //GPS模块
#define XTTCA_ADDR					0x66     //测控数传一体机A
#define XTTCB_ADDR					0x67     //测控数传一体机B
#define XJTX_ADDR					0x68     //星间通信
#define	DZB_ADDR					0x69	//搭载板
#define	XMA_ADDR					0x6a	//星敏A
#define	XMB_ADDR					0x6b	//星敏B
#define	FLX_ADDR					0x6c	//飞轮X
#define	FLY_ADDR					0x6d	//飞轮Y
#define	FLZ_ADDR					0x6e	//飞轮Z
#define	XKZ_ADDR					0x6f	//相控阵
#define	EP_ADDR						0x70	//电推
#define	FLS_ADDR					0x71	//飞轮S
#define	GSA_ADDR					0x72	//陀螺A
#define	GSB_ADDR					0x73	//陀螺B
#define	ZH1_ADDR					0x74	//载荷1
#define	ZH2_ADDR					0x75	//载荷2
#define	ZH3_ADDR					0x76	//载荷3
#define	DLX_ADDR					0x77	//动力学


/*----------------------------------------------------*/


enum RS422_MACHINE {XTTCA,XTTCB,TLA,TLB,GNSS,GNB,XMA,XMB,FLX,FLY,FLZ,FLS,PCDU,EP,GNSS_R,SBT_DCS,SJTX};
enum RS422_MACHINE2 {BAK1,BAK2,AOCC,DLX};

//CKJJM排在USBYDJ后面
//wxy:待修改.以上枚举类型待精修.是否有顺序需求?

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

//RS422新收机制
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
	CPU_INT32U flRealV;		//飞轮当前实际转速
	CPU_INT32U flRealCur;	//飞轮当前实际采样电流
	CPU_INT32U ctrlStat;	//控制状态
	CPU_INT32U accelerateTorque;	//飞轮加速力矩
	CPU_INT08U ID;			//错误指令计数
	CPU_INT08U sum;			//累加和
} FL_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U	APID;
	CPU_INT16U	flag;
	CPU_INT16U	screenGridVol;	//6-7屏栅电压U16
	CPU_INT16U	screenGridCur;	//8-9屏栅电流U16
	CPU_INT16U	accelerateGridVol;	//10-11加速栅电压U16
	CPU_INT16U	accelerateGridCur;	//12-13加速栅电流U16
	CPU_INT16U	heaterVol;	//14-15加热器电压U16
	CPU_INT16U	heaterCur;	//16-17加热器电流U16
	CPU_INT16U	neutralizerVol;	//18-19中和器电压U16
	CPU_INT16U	neutralizerCur;	//20-21中和器电流U16
	CPU_INT08U	microwaveSrcStat;	//22微波源状态
	CPU_INT16U	pressureSensor_1;	//23-24压力传感器1U16
	CPU_INT16U	pressureSensor_2;	//25-26压力传感器2U16
	CPU_INT16U	pressureSensor_3;	//27-28压力传感器3U16
	CPU_INT08U	mode;	//29运行模式
	CPU_INT08U	selfCheck;	//30自检
	CPU_INT08U	faultFeedback;	//31故障反馈
} EP_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
    CPU_INT16U  APID;       //2-3APID
    CPU_INT08U  sampling;       //4原始数据采样开关（黄河默认为0）
    //CPU_INT08U  mode;           //载荷工作模式
    CPU_INT08U  stateMachine;   //5FLASH控制的状态机监测量
    //CPU_INT08U  OC;             //DSP的OC切换响应
    CPU_INT16U   time[3];       //6-11GNSS时间（GNSS-R载荷生成）U16[3]
    CPU_INT08U   X[4];          //12-15GNSS定位位置X（GNSS-R载荷生成）S32
    CPU_INT08U   Y[4];          //16-19GNSS定位位置Y（GNSS-R载荷生成）S32
    CPU_INT08U   Z[4];          //20-23GNSS定位位置Z（GNSS-R载荷生成）S32
    CPU_INT08U   Vx[4];         //24-27GNSS定位速度Vx（GNSS-R载荷生成）S32
    CPU_INT08U   Vy[4];         //28-31GNSS定位速度Vy（GNSS-R载荷生成）S32
    CPU_INT08U   Vz[4];         //32-35GNSS定位速度Vz（GNSS-R载荷生成）S32
    CPU_INT08U  satCnt;         //36可见卫星数量
    //CPU_INT08U  status;         //定位标志（数据有效标志）
    //CPU_INT08U  pps;            //1PPS信号
    CPU_INT08U reserve[3];        //37-39Flash错误计数预留（全0）
} GNSS_R_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT16U	APID;	//2-3APID
	CPU_INT08U	channel_HighSpeedBit;	//4通道状态—高速位同步等
	CPU_INT08U	highSpeedChannelSNR;	//5高速信道SNR等
	CPU_INT08U	frequency1SNR;	//6频点1通道1-SNR等
	CPU_INT08U	frequency2SNR;	//7频点2通道1-SNR等
	CPU_INT08U	importantParamBackupCnt;	//8自身重要参数备份计数等
	CPU_INT16U	fixedMemoryRecordAddr;	//9-10固存记录地址
	CPU_INT16U	fixedMemoryPlaybackAddr;	//11-12固存回放地址
	CPU_INT08U	highSpeedAGC;	//13高速通道AGC
	CPU_INT08U	lowSpeedAGC;	//14低速通道AGC
	CPU_INT08U	operatingCur;	//15工作电流U8
	CPU_INT08U	pwrAmplifierCur;	//16功放电流U8
	CPU_INT08U	operatingVol;	//17工作电压U8
	CPU_INT08U	cpuVer;	//18处理器软件版本号等
	CPU_INT08U	generalMsgRxCnt;	//19普通报文接收计数等
	CPU_INT08U	ephemerisRxCnt;	//20星历上注接收计数等
	CPU_INT08U	cpuHeartbeatCnt;	//21处理器心跳计数等
	CPU_INT08U	lowSpeedInjectionCnt;	//22低速上注计数等
	CPU_INT08U	curFPGAProgramLoc;	//23当前FPGA程序位置等
	CPU_INT08U	internalBroadcastDownCnt;	//24内部状态—广播下行计数等
	CPU_INT08U	invalid422Cnt;	//25无效422指令计数等
	CPU_INT08U	businessDownExecutionCnt;	//26业务下行执行计数等
	CPU_INT08U	highSpeedRxCnt;	//27高速报文接收计数
	CPU_INT08U	fixedMemory422Playback;	//28A固存422顺序回放开/关等
	CPU_INT08U	businessRxCnt;	//29注册业务接收计数
	CPU_INT08U	starTime;	//30整星时间
	CPU_INT08U	refactorMode;	//31重构模式等
	CPU_INT08U	curRefactorAddr_15_8;	//32当前重构地址[15:8]
	CPU_INT08U	curRefactorAddr_7_0;	//33当前重构地址[7:0]
	CPU_INT08U	refactorAreaTotEraseCnt;	//34重构区全擦除计数等
	CPU_INT08U	CRC_BlockChkExecutionCnt;	//35CRC错误计数—块校验执行计数等
	CPU_INT08U	operationAddrErrCnt;	//36操作地址错误计数等
	CPU_INT08U	FLASHFrameHeadCorrCnt;	//37回读FLASH帧头正确计数等
	CPU_INT16U	refactorWrFLASHSucCnt;	//38-39重构数据写FLASH成功计数
	CPU_INT08U	highSpeedInjectionRxCnt;	//40高速上注接收计数
	CPU_INT08U	curRefactorAreaNum;	//41状态数据类型—当前重构区编号等
	CPU_INT08U	refactorStat_1;	//42重构状态1
	CPU_INT08U	refactorStat_2;	//43重构状态2
	CPU_INT08U	refactorStat_3;	//44重构状态3
	CPU_INT08U	refactorStat_4;	//45重构状态4
	CPU_INT08U	refactorStat_5;	//46重构状态5
	CPU_INT08U	refactorStat_6;	//47重构状态6
	CPU_INT08U	refactorStat_7;	//48重构状态7
	CPU_INT08U	refactorStat_8;	//49重构状态8
	CPU_INT08U	refactorStat_9;	//50重构状态9
	CPU_INT08U	refactorStat_10;	//51重构状态10
	CPU_INT08U	refactorStat_11;	//52重构状态11
	CPU_INT08U	refactorStat_12;	//53重构状态12
	CPU_INT08U	refactorStat_13;	//54重构状态13
	CPU_INT08U	refactorStat_14;	//55重构状态14
	CPU_INT08U	refactorStat_15;	//56重构状态15
	CPU_INT08U	refactorStat_16;	//57重构状态16
} SBT_DCS_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U	operatingVol;	//2工作电压U8
	CPU_INT08U	basebandCur;	//3基带电流U8
	CPU_INT08U	AGC;	//4接收AGC
	CPU_INT08U	pwrAmplifierCur;	//5功放电流U8
	CPU_INT08U	pwrAmplifierTemp;	//6功放温度U8
	CPU_INT08U	reserve[3];	//7-9保留
	CPU_INT08U	workStat_UpMode;	//10上行工作状态等
	CPU_INT08U	channel_RxFirstLevel;	//11接收一级本振锁定等
	CPU_INT08U	channelWork_DownModulation;	//12下行调制开关等
	CPU_INT08U	rcChannelSNR;	//13遥控通道SNR
	CPU_INT08U	rcDirectCmdCnt;	//14遥控直接指令计数(保留)
	CPU_INT08U	rcInjectionCmdCnt;	//15遥控上注指令计数(保留)
	CPU_INT08U	indirectCmdCnt_Rx;	//16间接指令接收计数等
	CPU_INT08U	selfCheck1_RepairableCnt;	//17调制器可修复翻转计数等
	CPU_INT08U	selfCheck2_reserve1;	//18重配置使能等
	CPU_INT08U	ddt_CannotFixErr;	//19数传Flash读取数据TMR无法修复错误标记等
	CPU_INT16U	storageAddr;	//20-21存储地址指针
	CPU_INT16U	sequentialPlayback;	//22-23顺序回放指针
	CPU_INT16U	curPlayback;	//24-25当前回放指针
	CPU_INT08U	fixedMemory_Stat;	//26固存状态等
	CPU_INT08U	loadFrameCnt;	//27载荷传输帧接收计数
	CPU_INT08U	telemetryFrameRxCnt;	//28遥测帧接收计数
	CPU_INT08U	downTelemetryFrameCnt;	//29下行遥测帧计数
	CPU_INT08U	injectionCorrFrameCnt;	//30上行上注数据正确帧计数
	CPU_INT08U	injectionErrFrameCnt;	//31上行上注数据错误帧计数
	CPU_INT08U	reserve_2;	//32-33保留
	CPU_INT08U	reserve_3;	//32-33保留
	CPU_INT08U	monitorFPGAVer;	//34监控FPGA版本
	CPU_INT08U	internalTestVerNum;	//35软件内测版本号
	CPU_INT08U	verNum;	//36软件版本号
	CPU_INT08U	check;	//37校验和
} XTTC_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U solid;		//固定值
	CPU_INT08U att4Mmt[16];	//姿态四元数
	CPU_INT08U innTime[7];	//星敏感器内部时间
	CPU_INT08U temp;		//星敏感器成像传感器温度
	CPU_INT08U picExp;		//图像曝光值
	CPU_INT08U picLmt;		//图像阈值
	CPU_INT08U bkgrd;		//背景值
	CPU_INT08U innSign;		//EDAC开关标志/FPGA_MRAM自检结果/CMOS Chip ID检验结果/系统内部工作进程代号
	CPU_INT08U innPara;		//系统工作模式/提取星数
	CPU_INT08U gainDb;		//四元数有效时导航星数/增益大小
	CPU_INT08U dataValid;	//识别星数/外部图像传输打开关断标识/数据有效标志
	CPU_INT08U devNo;		//内部软件版本号/产品设备编号
	CPU_INT08U EDACErrCnt;		//EDAC错误计数
	CPU_INT08U picNo[3];		//图像帧号
	CPU_INT08U Att4MmtSwi;	//SAA阈值/SAA工作模式/四元数滤波开关标志位
	CPU_INT08U starLmt;		//四星寻找阈值
	CPU_INT08U followLmt;	//跟踪阈值
	CPU_INT08U sum;			//累加和
} XM_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U TC_state;        //遥控通道状态
	CPU_INT08U TC_carr_devia;   //遥控通道载波频偏
	CPU_INT08U TC_SNR;          //遥控通道SNR
	CPU_INT08U TC_AGC;          //遥控通道AGC
	CPU_INT08U Refresh_state;   //刷新状态
	CPU_INT08U sum;	            //和校验
} XKZ_TM;
#pragma pack()

#pragma pack(1)
typedef struct
{
	CPU_INT08U st[499];
	CPU_INT08U sum;	            //和校验
} DLX_TM;
#pragma pack()


#endif
//GPS1_TM_TAB GPS模块回送格式1:1s一次

#pragma pack(1)
typedef struct
{
	CPU_INT16U frame_type;          //帧类型0x1001
	CPU_INT16U data_word_length;    //有效数据字长度447
	CPU_INT32U TIC_cnt;             //GPS开机时间(TIC计数)
	CPU_INT08U gps_state;           //GPS定位状态
	CPU_INT08U gps_satellite_cnt;   //GPS定位卫星数
	CPU_INT16U GPS_week;            //GPS周
	CPU_INT32U GPS_sec;             //GPS周秒
	CPU_INT16U BDS_week;            //BD周:wxy新的通信协议新增
	CPU_INT32U BDS_sec;             //BD周秒
	CPU_INT32U UTC_sec;            //UTC时间累积秒值
	CPU_INT32S gps_location[3];               //GPS定位位置X,Y,Z
	CPU_INT32S gps_V[3];              //GPS定位速度Vx,Vy,Vz
	CPU_INT32S gps_clock_err;       //GPS钟差
	CPU_INT32S gps_clock_drift;     //GPS钟漂
	CPU_INT16U gps_PDOP;
	CPU_INT16U gps_GDOP;

	CPU_INT08U L1_chan1_satellite_num;  //L1通道1卫星号
	CPU_INT08U L1_chan1_state;          //L1通道1状态
	CPU_INT08U L1_chan1_SN;             //L1通道1信噪比
	CPU_INT08U L1_chan1_ELV;            //保留（固定为0）
	CPU_INT32U L1_chan1_wj_integer;     //L1通道1伪距整数部分
	CPU_INT08U L1_chan1_wj_decimal;     //L1通道1伪距小数部分
	CPU_FP32   L1_chan1_DP;             //L1通道1载波多普勒值
	CPU_INT32S L1_chan1_cw_integer;     //L1通道1载波相位整数部分
	CPU_INT16S L1_chan1_cw_decimal;     //L1通道1载波相位小数部分

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

	CPU_INT08U B1_chan1_satellite_num;  //B1通道1卫星号
	CPU_INT08U B1_chan1_state;          //B1通道1状态
	CPU_INT08U B1_chan1_SN;             //B1通道1信噪比
	CPU_INT08U B1_chan1_ELV;            //B1通道1卫星仰角wxy
	CPU_INT32U B1_chan1_wj_integer;     //B1通道1伪距整数部分
	CPU_INT08U B1_chan1_wj_decimal;     //B1通道1伪距小数部分
	CPU_FP32   B1_chan1_DP;             //B1通道1载波多普勒值
	CPU_INT32S B1_chan1_cw_integer;     //B1通道1载波相位整数部分
	CPU_INT16S B1_chan1_cw_decimal;     //B1通道1载波相位小数部分

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

    CPU_INT08U curWorkModule;//444导航模块工作状态
	CPU_INT08U version;  	//445软件版本号
    CPU_INT08U orbitStat;	//446定轨状态-保留
	CPU_INT08U reserved[64];//保留
	CPU_INT08U sum;         //和校验
} GNSS_TM;
#pragma pack()


///*-------------------GNB_TM_Tab功能板遥测表---------------------------------*/
#pragma pack(1)
typedef struct
{
#if 1
	//CPU_INT16U syn;				//0-1同步字
	CPU_INT16U frameCnt;               //2-3遥测帧计数
    CPU_INT16U frameLen;               //4-5帧长
    CPU_INT08U cmdCnt;                  //6指令计数
    CPU_INT08U cmdRxStat;               //7指令接收状态
    CPU_INT16S sunAngle_a1;          //8-9太阳角计a1
    CPU_INT16S sunAngle_a2;          //10-11太阳角计a2
    CPU_INT16S sunAngle_a3;          //12-13太阳角计a3
    CPU_INT16S sunAngle_a4;          //14-15太阳角计a4
    CPU_INT16S sunAngle_b1;          //16-17太阳角计b1
    CPU_INT16S sunAngle_b2;          //18-19太阳角计b2
    CPU_INT16S sunAngle_b3;          //20-21太阳角计b3
    CPU_INT16S sunAngle_b4;          //22-23太阳角计b4
    CPU_INT16U curBackup_1;            //24-25电流备用1
    CPU_INT16U curBackup_2;            //26-27电流备用2
    CPU_INT16U curBackup_3;            //28-29电流备用3
    CPU_INT16U curBackup_4;            //30-31电流备用4
    CPU_INT16U curBackup_5;            //32-33电流备用5
    CPU_INT16U curBackup_6;            //34-35电流备用6
    CPU_INT16U curBackup_7;            //36-37电流备用7
    CPU_INT16U curBackup_8;            //38-39电流备用8
    CPU_INT16U maskTemp_PosX;          //40-41+X板多层面膜温度
    CPU_INT16U maskTemp_NegZ;          //42-43-Z板多层面膜温度
    CPU_INT16S satSeparated_2;         //44-45星箭分离信号2
    CPU_INT16S busVoltage;           //46-47母线电压遥测
    CPU_INT16S batteryVoltage;       //48-49蓄电池组电压遥测
    CPU_INT16S loadCurrent;          //50-51负载电流遥测
    CPU_INT16S solarArrayCur;        //52-53太阳阵电流遥测
    CPU_INT16S batteryDischargeCur;  //54-55蓄电池组放电电流遥测
    CPU_INT16S batteryChargeCur;     //56-57蓄电池组充电电流遥测
    CPU_INT16S GNSS_RReceiver;       //58-59GNSS-R接收机工作电压
    CPU_INT16S GNSS_RAmplifier;      //60-61GNSS-R低噪放供电电压
    CPU_INT16S magnetometer_X;       //62-63磁强计X磁场强度
    CPU_INT16S magnetometer_Y;       //64-65磁强计Y磁场强度
    CPU_INT16S magnetometer_Z;       //66-67磁强计Z磁场强度
    CPU_INT16S FPGAMainDownload;       //68-69FPGA主程序加载指示
    CPU_INT16S FPGABackupDownload;     //70-71FPGA备程序加载指示
    CPU_INT16S DSPDownload;            //72-73DSP程序加载完成指示
    CPU_INT16U reserve;                //74-75备用
    CPU_INT16U batteryTemp_1;        //76-77蓄电池组温度遥测1
    CPU_INT16U batteryTemp_2;        //78-79蓄电池组温度遥测2
    CPU_INT16U receiverTemp;         //80-81GNSS-R接收机温度
    CPU_INT16U upAntennaTemp;        //82-83上视天线低噪放温度
    CPU_INT16U downAntennaTemp;    //84-85下视天线低噪放温度
    CPU_INT16U downAntennaTemp_2;    //86-87下视天线低噪放温度2
    CPU_INT16U SSATemp;           //88-89星敏感器A温度
    CPU_INT16U SSBTemp;           //90-91星敏感器B温度
    CPU_INT16S FPGADownload;           //92-93FPGA程序加载完成指示
    CPU_INT16U volBackup_2;            //94-95电压备份2
    CPU_INT16U volBackup_3;            //96-97电压备份3
    CPU_INT16U volBackup_4;            //98-99电压备份4
    CPU_INT16U GSTemp;           //100-101陀螺温度
    CPU_INT16U clapboardTemp;       //102-103隔板框架温度
    CPU_INT16U FWTriTemp;           //104-105三轴飞轮温度
    CPU_INT16U FWSTemp;           //106-107单轴飞轮温度
		/*21加热器温度*/
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

		
    CPU_INT16U battTemp_1_HeatCtrl;           //108-109蓄电池温度1-热控
    CPU_INT16U battTemp_2_HeatCtrl;           //110-111蓄电池温度2-热控
    CPU_INT16U OBCTemp;           //112-113综合电子温度
    CPU_INT16U XTTCATemp;          //114-115射频基带（主）温度
    CPU_INT16U XTTCBTemp;          //116-117射频基带（备）温度
    CPU_INT16U PCDUTemp;          //118-119电源控制器温度
    CPU_INT16U DCSTemp;          //120-121DCS载荷温度
    CPU_INT16U downAntennaCombinerTemp;          //122-123GNSS-R下视天线合路器温度
    CPU_INT16U maskTemp_NegY;          //124-125-Y板多层面膜温度
    CPU_INT16U V24Power;               //126-127V24配电
    CPU_INT16U V25Power;               //128-129V25配电
    CPU_INT16S V11Power;               //130-131V11配电（GPS配电）
    CPU_INT16U V14Power;               //132-133V14配电
    CPU_INT16S CPUAPower;              //134-135CPUA配电
    CPU_INT16S CPUBPower;              //136-137CPUB配电
    CPU_INT16S CPUBSignal;             //138-139CPUB权信号
    CPU_INT16S busbar_5V;            //140-141母线5V
    CPU_INT16S busbar_12V;           //142-143母线12V
    CPU_INT16U busbar_28V;           //144-145母线28V
    CPU_INT16S Magnetorquer_Pos1;    //146-147磁力矩器1+电流
    CPU_INT16S Magnetorquer_Neg1;    //148-149磁力矩器1-电流
    CPU_INT16S Magnetorquer_Pos2;    //150-151磁力矩器2+电流
    CPU_INT16S Magnetorquer_Neg2;    //152-153磁力矩器2-电流
    CPU_INT16S Magnetorquer_Pos3;    //154-155磁力矩器3+电流
    CPU_INT16S Magnetorquer_Neg3;    //156-157磁力矩器3-电流
    CPU_INT16U temp;                 //158-159综合电子单机温度
    CPU_INT16U satSeparated_1;          //160星箭分离信号1
		
		
    /*CPU_INT08U heater_15;               //加热器15开关
    CPU_INT08U heater_14;               //加热器14开关
    CPU_INT08U heater_13;               //加热器13开关
    CPU_INT08U heater_12;               //加热器12开关
    CPU_INT08U heater_11;               //PCDU加热器开关
    CPU_INT08U heater_10;               //星敏B加热器开关
    CPU_INT08U heater_9;                //星敏A加热器开关
    CPU_INT08U heater_8_1;              //161电推加热器开关
    CPU_INT08U heater_7;                //GNSS下视低噪放加热器开关
    CPU_INT08U heater_6;                //GNSS上视低噪放加热器开关
    CPU_INT08U heater_5;                //GNSS合路器加热器开关
    CPU_INT08U heater_4;                //GNSS中频接收机加热器开关
    CPU_INT08U heater_3;                //综合电子加热器开关
    CPU_INT08U heater_2;                //蓄电池加热器备开关
    CPU_INT08U heater_1;                //蓄电池加热器主开关*/
    CPU_INT16U end_1;                  //162-163遥测结束标志AABB
    CPU_INT16U end_2;                  //164-165遥测结束标志CCDD
    CPU_INT08U end_3;                   //166遥测结束标志EE
    CPU_INT08U check;                   //167异或校验
#endif
} GNB_TM;
#pragma pack()
#pragma pack(1)
typedef struct
{
	//CPU_INT32Usyn;	//0~3同步字0xeb90000f，原来没有
	CPU_INT16U	busVoltage;	//4-5母线电压遥测
	CPU_INT16U	batteryVoltage;	//6-7蓄电池电压遥测
	CPU_INT16U	solarArrayCur;	//8-9太阳阵电流遥测
	CPU_INT16U	batteryDischargeCur;	//10-11蓄电池组放电电流遥测
	CPU_INT16U	batteryChargeCur;	//12-13蓄电池组充电电流遥测
	CPU_INT16U	loadCurrent;	//14-15负载电流遥测
	CPU_INT16U	voltage_5_2V;	//16-175.2V电压遥测
	CPU_INT16U	voltage_12V;	//18-1912V电压遥测
	CPU_INT16U	voltage_neg12V;	//20-21-12V电压遥测
	CPU_INT16U	voltage_6V;	//22-236V电压遥测
	CPU_INT16U	solarArrayTemp_PosX;	//24-25+X翼帆板温度
	CPU_INT16U	solarArrayTemp_NegX;	//26-27-X翼帆板温度
	CPU_INT16U	temperature_1;	//28-29PCDU温度
	CPU_INT16U	reserve;	//30-31预留
	CPU_INT08U	dischargeSwStat;	//32放电开关状态等
	CPU_INT08U	GSAPwrStat;	//33陀螺A供电状态等
	CPU_INT08U	EP3PwrStat;	//34电推控制配电3供电状态等
	CPU_INT08U	receiverPwrStat_Neg12V;	//35接收机-12V供电状态等
	CPU_INT08U	hotKnifeFlightPlugStat_2;	//36热刀飞行插头2状态等
	CPU_INT16U	currentBattery;	//37-38当前电量
	CPU_INT08U	instructionexecutionCnt;	//39指令执行计数
	CPU_INT08U	warmResetCnt;	//40热复位计数
	CPU_INT08U	fullAmpereHourVol;	//41上注蓄电池组满安时参数(蓄电池电压)
	CPU_INT08U	fullAmpereHourCur;	//42上注蓄电池组满安时参数(蓄电池电流)
	CPU_INT08U	internalVer;	//43软件内部版本
	CPU_INT08U	lowerCpuStat;	//44下位机主备状态
	CPU_INT08U	reserve_2[18];	//45-62预留
	CPU_INT08U	check;	//63校验和

} PCDU_TM;
#pragma pack()

//-------------------------AOCC内结构体
// 陀螺遥测数据包(16 Byte)
#pragma pack(1)
typedef struct
{
	unsigned char GuSw; 				// 陀螺数据状态字
	short HeaderDriftX; 				// 陀螺X向表头漂移
	short HeaderDriftY; 				// 陀螺Y向表头漂移
	short HeaderDriftZ; 				// 陀螺Z向表头漂移
	unsigned char GuSpdX[3];			// 陀螺X轴角速度
	unsigned char GuSpdY[3];			// 陀螺Y轴角速度
	unsigned char GuSpdZ[3];			// 陀螺Z轴角速度
	short GuTempX;
	short GuTempY;
	short GuTempZ;
	short bk;
}AOCC_GURM;
#pragma pack()

// 星敏遥测数据包(36 Byte)
#pragma pack(1)
typedef struct
{
	float StQsi[4];					    // ST姿态四元数
	unsigned int StTimeStampS;			// ST内部时间秒
	short StSpd[3];						// ST伪速率
	unsigned char StTimeStampMS[3];		// ST内部时间毫秒
	char StCCDTemp;						// STCCD温度
	char StRateQuality;					// ST速率质量
	char StWkSwA;						// ST增益大小/ST1四元数有效时导航星数
	char StWkSwB;						// ST单机数据有效标识/ST1外部图像传输标识/ST1识别星数
	char StWkSwC;						// ST四元数滤波开关/ST1SAA工作模式/ST1SAA阈值
	char StBgValue;						// ST背景值
	char bk;					
	short StDiffTime;					// 星敏曝光时间差值
    short StSunAng;
    short StEthAng;
    short bk2;

}AOCC_STRM;
#pragma pack()

// 磁强计遥测数据包(6 Byte)
#pragma pack(1)
typedef struct
{
	short MagMm[3];						// 磁强计测量值
	short bk;
}AOCC_MMRM;
#pragma pack()


///*-------------------AOCC_TM_Tab姿轨控遥测表---------------------------------*/
//#pragma pack(1)
//typedef struct
//{
//		CPU_INT08U syn[2]; 				
//		CPU_INT16U TM_frame_cnt;			// 姿轨控遥测帧计数

//		// 姿态遥测
//		unsigned char AttCtlDiffX[3];		// X向姿态角控制偏差
//		unsigned char AttCtlDiffY[3];		// Y向姿态角控制偏差
//		unsigned char AttCtlDiffZ[3];		// Z向姿态角控制偏差
//		unsigned char AttCtlGlobalDiffX[3]; // X向姿态角全局控制偏差
//		unsigned char AttCtlGlobalDiffY[3]; // Y向姿态角全局控制偏差
//		unsigned char AttCtlGlobalDiffZ[3]; // Z向姿态角全局控制偏差
//		unsigned char UVecSatSunBX[3];		// X轴本体系下卫星指向太阳的单位矢量
//		unsigned char UVecSatSunBY[3];		// Y轴本体系下卫星指向太阳的单位矢量
//		unsigned char UVecSatSunBZ[3];		// Z轴本体系下卫星指向太阳的单位矢量
//		unsigned char IneSpdX[3];			// X轴惯性角速度
//		unsigned char IneSpdY[3];			// Y轴惯性角速度
//		unsigned char IneSpdZ[3];			// Z轴惯性角速度
//		short SpdCtlDiffX;					// X向角速度控制偏差
//		short SpdCtlDiffY;					// Y向角速度控制偏差
//		short SpdCtlDiffZ;					// Z向角速度控制偏差
//		short SpdCtlGlbDiffX;				// X轴全局角速度控制偏差
//		short SpdCtlGlbDiffY;				// Y轴全局角速度控制偏差
//		short SpdCtlGlbDiffZ;				// Z轴全局角速度控制偏差
//		unsigned short SailSunAng;			// 帆板太阳角
//		short cosASInteSum; 				// 太阳角余弦积分
//		short MagTbl[3];					// 磁场表数据
//		short MagInUse[3];					// 实际使用的磁场数据  
//		float AttCfmQuat[4];				// 本体系相对惯性系的四元数
//		float AssAngf[3];					// 模拟太阳角计太阳矢量
//		float ObtPhsAng;					// 低倾角对日卫星轨道相位角
//		float ObtPhsAngStr; 				// 低倾角对日导引卫星轨道相位角
//		float ObtSunAng;					// 低倾角对日轨道太阳角
//		float DiffQuat[3];					// 误差四元数
//		float Qir[4];						// 规划四元数
//		short Wrr[3];						// 规划角速度
//		short AssAngM[3]; 					// 模拟太阳角计算出的太阳角
//		short AssAngQ[3];					// 姿态计算出的太阳角
//		short bkt21;
//		float QIEnd[4];						// 目标四元数
//		float AngObt[3];					// 轨道系姿态角
//		
//		// 控制遥测
//		short StarAngMmt[3];				// 整星角动量
//		short FeedForwardMmt[3];			// 前馈力矩
//		short PIDCtrlMmt[3];				// PID控制力矩
//		short RwSpdCmd[4];					// 飞轮指令转速
//		short MtCmdMmt[3];					// 磁力矩器轴指令磁矩
//		short Tdw[3];						// 姿态机动角加速度前馈力矩 	
//		short Tmmt[3];						// 误差角动量反馈力矩
//		short Tquat[3]; 					// 误差四元数反馈力矩
//		short Tq0inte[3];					// 误差四元数标部积分反馈力矩
//		short StarAngMmtB[3];				// 本体系下整星角动量
//		short bkc0;

//		// 机动遥测
//        unsigned char MoveSdyState; 			// 机动稳态状态
//        unsigned char CmdLockStatus;			// 指令锁状态
//        unsigned char TaskRTPlanStatus;				// 任务实时拒收状态
//        unsigned char TaskWaitNum;				// 姿控指令待发计数
//        unsigned char WkStartTime[4];			// 当前工作开始时间
//        unsigned char WkEndTime[4];				// 当前工作结束时间
//        unsigned char NextWkStaTime[4];			// 下一条指令开始时间
//        unsigned char NextWkEndTime[4];			// 下一条指令结束时间
//        unsigned char TaskHtRejectTime[4];		// 历史拒收时刻
//        unsigned short TaskUploadNum;			// 姿控指令上注计数
//        unsigned short TaskInvalidNum;			// 姿控指令无效计数
//        unsigned short TaskPushNum;				// 姿控指令入队计数
//        unsigned short TaskRejectNum;			// 姿控指令拒收计数
//        unsigned short TaskExcuteNum;			// 姿控指令执行计数
//        unsigned short TaskClearNum;			// 姿控指令清除计数
//        unsigned short TaskGeneNum;				// 姿控指令自主生成计数
//		short bke2;
//        unsigned char TaskHtReject;				// 历史拒收状态
//        unsigned char  TaskAcceptStatus;		// 给星务拒收标志
//		unsigned short CurrentCmdId;			// 当前指令号
//		unsigned char CurrentCordSys[2];		// 当前坐标系
//		unsigned char staMode[2]; 				// 当前任务起始坐标系
//		unsigned short TargetInfoNum;			// 弹道包数目
//		short MoveEulAngSin;					// 四元数机动欧拉轴角sin函数幅值
//		unsigned short PlQtClimbTime;			// 规划四元数爬升段时间t1
//		unsigned short PlQtConstTime;			// 规划四元数匀速段时间t2
//		unsigned short PlSpSlowTime;			// 规划角速度减速段时间t4
//		unsigned short PlSpAccTime; 			// 规划角速度加速段时间t5
//		short BaseSpdPlan[3];					// 基准规划角速度
//		short MoveSpdPlan[3];					// 机动规划角速度
//		short MoveEulerAxis[3]; 				// 四元数机动规划欧拉轴
//		short MoveEulerAng; 					// 四元数机动规划欧拉轴角 
//		unsigned short MoveEulerSpdMax; 		// 规划欧拉轴角最大角速度
//		unsigned short MoveTimePlan;			// 规划机动总时间

//	
//		// 轨控任务遥测
//		unsigned char ObtLockStatus;			// 指令锁状态
//		unsigned char ObtTaskWaitNum;			// 轨控指令待发计数
//		unsigned short ObtTaskUploadNum;		// 轨控指令上注计数
//		unsigned short ObtTaskInvalidNum;		// 轨控指令无效计数
//		unsigned short ObtTaskManuNum;			// 轨控指令规划计数
//		unsigned short ObtTaskPushNum;			// 轨控指令入队计数
//		unsigned short ObtTaskRejectNum;		// 轨控指令拒收计数
//		unsigned short ObtTaskClearNum; 		// 轨控指令清除计数
//		char ObtRejectStatus;					// 轨控指令拒收状态
//		unsigned char ThrustEnable; 			// 推进使能标志
//		unsigned char JetStaTime[4];			// 喷气指令开始时刻
//		unsigned char JetEndTime[4];			// 喷气指令结束时刻
//		unsigned char JetNextStaTime[4];		// 下一条喷气指令开始时刻
//		unsigned char JetNextEndTime[4];		// 下一条喷气指令结束时刻
//		unsigned short JetPushNum;				// 喷气队列入队计数
//		unsigned char JetWaitNum;				// 喷气队列待发计数
//		unsigned short JetExcuteNum;			// 喷气队列执行计数
//		unsigned short JetClearNum; 			// 喷气队列清除计数
//		unsigned char ObtCtrlEnble; 			// 轨控使能开关
//		unsigned char ObtCtrlInProcess; 		// 轨控进行中标志
//		char bkc2[3];

//		// 轨道遥测
//		float SemiMajorAxis;				// 轨道半长轴
//		float EccenRatio;					// 轨道偏心率
//		float DeclAngle;					// 轨道轨道倾角
//		float LatiArgu; 					// 轨道纬度幅角
//		float AscenNode;					// 轨道升交点赤经
//		float PerigeeArgu;					// 轨道近地点幅角
//		float MA;							// 轨道平近点角
//		float FA;							// 轨道真近点角
//		float ObtSpd;						// 轨道角速度
//		float SrcSemiMajorAxis; 			// 初始轨道递推半长轴
//		float SrcEccenRatio;				// 初始轨道递推偏心率
//		float SrcDeclAngle; 				// 初始轨道递推轨道倾角
//		float SrcLatiArgu;					// 初始轨道递推升交点赤经
//		float SrcAscenNode; 				// 初始轨道递推近地点幅角
//		float SrcPerigeeArgu;				// 初始轨道递推纬度幅角
//		float SrcMA;						// 初始轨道递推平近点角
//		float GPSWgsPos[3]; 				// WGS位置
//		float GPSWgsSpd[3]; 				// WGS速度
//		unsigned char SrcRefClk[4];			// 初始轨道历元
//		unsigned char ObtCalTime[4];		// 轨道计算时刻
//		unsigned char ObtCalUw; 			// 轨道数据接入状态
//		unsigned char ObtDtStatus;			// 轨道递推状态
//		unsigned char ObtUwSelect;			// 地面上注使用轨道数据
//		unsigned char ObtDtType;				// 地面上注数据类型

//		// 模式状态 遥测
//		unsigned short AocsStatus;				// AOCS状态字
//		unsigned short SysModSet;			// 系统设置模式
//		unsigned char SunshineFlag; 			// 由姿态算出的光照区标志
//		unsigned char SysWkMod; 				// 工作模式
//		unsigned char Breakaway;				// 星箭分离信号
//		unsigned char StSelection;				// 星敏感器选择标志
//		unsigned char GuSelection;				// 陀螺选择标志
//		unsigned char AttInPlaceFlag;			// 稳态/机动标志
//		unsigned char RcPkgId;					// 遥控包ID类型
//		unsigned char RcPkgErrId;				// 遥控包数据检查
//		unsigned short RcPkgCnt;				// 遥控包上注计数
//		unsigned char SunCptr;					// 捕获太阳标志
//		unsigned char AocsRunTime;				// 软件实际运行时长
//		unsigned char timeJcCnt;				// 综电时间跳变计数
//		unsigned char memExceptCnt; 			// MARM3取2异常计数
//		unsigned char MagUseMod;				// 磁场计算方式
//		unsigned char RwSelection;				// 飞轮组合选择
//		unsigned short memExceptAddr;			// MRAM内存异常地址偏移
//		unsigned short MagDiffCnt;				// 磁强计不一致性计数
//		unsigned char SunshineFlagM;			// 模太得到的光照区标志
//		unsigned char SimRw;					// 使用模拟飞轮
//		unsigned char SimMt;					// 使用模拟磁棒
//		unsigned char DlxOn;					// 动力学模式标志
//		unsigned char AttBase;					// 姿态基准
//		unsigned char SpdBase;					// 角速度基准
//		unsigned char ForceMoveFlag;			// 强拉标志
//		unsigned char PwrLowNum;				// 能源危机次数
//		unsigned short PwrLowStatus;			// 能源危机标志
//		unsigned char SimSt;					// 模拟星敏
//		unsigned char SimGu;					// 模拟陀螺
//		unsigned char DlxAddOn; 				// 动力学累加模式
//		unsigned char SunAngleMode; 			// 太阳角计算方式
//		unsigned char GuInteOn; 				// 陀螺积分开关
//		unsigned char RwSelSet; 				// 系统设置飞轮组合
//		unsigned char SimAss;					// 模拟太敏
//		unsigned char SimMag;					// 模拟磁强计
//		unsigned char PpFtMonOn;				// 电推故障监控使能
//		unsigned char PpFtHandOn;				// 电推故障处理使能
//		int MagDampHealthCnt;					// 磁阻尼健康模式计数
//		int SailSunCnt; 						// 帆板太阳角超差计数
//		int SafeMdHealthCnt;					// 安全模式健康计数
//		int SrvModHealthCnt;					// 业务模式健康计数
//		unsigned char DlxInsideOn;				// 内闭环标志使能
//		unsigned char DlxFastRmOn;				// 动力学快速遥测使能
//		unsigned char StSelSet; 				// 地面上注使用星敏组合
//		unsigned char StKeep;					// 地面上注星敏保持
//		unsigned short DftCordSet;				// 地面上注待机坐标系
//		unsigned char MagSelSet;				// 地面上注使用磁场基准
//		unsigned char GuSelSet; 				// 地面上注使用陀螺基准
//		unsigned char GuKeep;					// 地面上注陀螺保持
//		unsigned char TaskLockEnble;			// 机动任务锁使能
//		unsigned char StAccept; 				// 星敏不可用强制接收
//		unsigned char StOnoff;					// 星敏可用标志
//		unsigned char SimGps;					// 模拟GPS
//		unsigned short DlxPkgNum;				// 动力学收包计数
//		unsigned char ObdhRmNC;                 // 星务数据包数据不变标志
//   	    unsigned short RwEmergyRemainCnt;		// 飞轮紧急处理尚存次数
//		unsigned short DelAttUpateCnt;			// 星敏姿态误差更新计数
//		unsigned short GuIntCnt;                // 陀螺积分使用计数
//		unsigned char TimeMatain;				// 时间维护开关
//		unsigned char bk1;

//		// 太阳角计
//		short AssCura[4];					// 模拟太阳角计a1-4电流
//		short AssCurb[4];					// 模拟太阳角计b1-4电流

//		// 飞轮遥测
//        short RwSpd[4];						// 飞轮转速
//        unsigned short RwCur[4];			// 飞轮电流
//		short RwRealSpd[4]; 				// 飞轮转速记录

//		// 喷气遥测
//		unsigned char PpPowerOnNum; 			// 喷气功率上电计数
//		unsigned char PpCtrlOnNum;				// 喷气控制上电计数
//		unsigned char PpGasEnbleNum;			// 喷气气瓶故障监控使能计数
//		unsigned char PpPressEnbleNum;			// 喷气压调故障监控使能计数
//		unsigned char PpBangEnbleNum;			// 喷气Bang故障监控使能计数
//		unsigned char PpAutoNum;				// 喷气自主飞行程序计数
//		unsigned char PpElectOffNum;			// 喷气电推进关机计数
//		unsigned char PpPowerOffNum;			// 喷气功率断电计数
//		unsigned char PpCtrlOffNum; 			// 喷气控制断电计数
//		unsigned char PpFaultStatus;			// 喷气历史故障状态
//		unsigned char PpWkMod;					// 喷气实时故障状态
//		char bk;
//		unsigned int PpFaultTime;				// 喷气历史故障时间


//		// 磁力矩器遥测
//		short MtCmdCur[3];					// 磁力矩器指令电流
//		short MtReColCur[3];				// 磁力矩器反采电流（X）

//		// 故障遥测
//		unsigned char GuFt[MAXGUNUM];		// 机械陀螺/光纤陀螺故障状态
//		unsigned char GuWkMod[MAXGUNUM];	// 机械陀螺/光纤陀螺单拍数据状态
//		unsigned char GuFtCnt[MAXGUNUM];	// 陀螺故障次数
//		unsigned char GuOnOffNum[MAXGUNUM];	// 陀螺开关机次数
//		unsigned char StFt[MAXSTNUM];		// 星敏故障状态
//		unsigned char StWkMod[MAXSTNUM];	// 星敏单拍数据状态
//		unsigned char StFtNum[MAXSTNUM];	// 星敏故障次数
//		unsigned char StOnOffNum[MAXSTNUM];	// 星敏开关机次数	
//		unsigned char RwFt[4];				// 飞轮故障状态
//		unsigned char RwWkMod[4];			// 飞轮单拍数据状态
//		unsigned char RwFtNum[4];			// 飞轮故障次数
//		unsigned char RwOnOffNum[4];		// 飞轮开关机次数
//		unsigned char MmFt[MAXMMNUM];		// 磁强计故障状态
//		unsigned char MmWkMod[MAXMMNUM];	// 磁强计单拍数据状态
//		unsigned char MmFtNum[MAXMMNUM];	// 磁强计故障次数
//		unsigned char MmOnOffNum[MAXMMNUM];	// 磁强计开关机次数
//		unsigned char GpsWkMod;				// GPS单拍数据情况
//		unsigned char GuFtDiag[MAXGUNUM];  	// 陀螺错误诊断开关
//		unsigned char StFtDiag[MAXSTNUM];	// 星敏错误诊断开关
//		unsigned char RwFtDiag[4];			// 飞轮故障诊断开关
//		unsigned char MmFtDiag[MAXMMNUM];	// 磁强计故障诊断开关
//		unsigned char GpsFtDiag;			// GPS故障诊断开关	
//		unsigned char ObtDtDiag;			// 地面上注轨道数据故障诊断开关
//		unsigned char bkf9[3];
//		unsigned char GuLastErrMod[MAXGUNUM];// 陀螺最近错误状态
//		unsigned char StLastErrMod[MAXSTNUM];// 星敏最近错误状态
//		unsigned char RwLastErrMod[4];		// 飞轮最近错误状态
//		unsigned char MmLastErrMod[MAXMMNUM];// 磁强计最近错误状态
//		unsigned char GpsLastErrMod;		// GPS最近错误状态
//		unsigned char ObtDtWkMod;			// 地面上注轨道数据错误状态
//		unsigned int StLastErrTime[MAXSTNUM];// 星敏最近错误时刻
//		unsigned int GuLastErrTime[MAXGUNUM];// 陀螺最近错误时刻
//		unsigned int MmLastErrTime[MAXMMNUM];// 磁强计最近错误时刻
//		unsigned int RwLastErrTime[4];		// 飞轮最近错误时刻
//		unsigned int GpsLastErrTime;		// GPS最近错误时刻
//		short GuHealthScore[MAXGUNUM];		// 陀螺健康积分
//		short StHealthScore[MAXSTNUM];		// 星敏健康积分
//		short RwHealthScore[4];				// 飞轮健康积分
//		short MmHealthScore[MAXMMNUM];		// 磁强计健康积分  

//		// 陀螺、星敏、磁强计遥测
//		AOCC_GURM	RmGu[MAXGUNUM];
//		AOCC_STRM	RmSt[MAXSTNUM];
//		AOCC_MMRM	RmMag[MAXMMNUM];

//		CPU_INT08U  sum;

//} AOCC_TM;

//#pragma pack()




////星务软件数据接口
//#pragma pack(1)
//typedef struct					//定义业务注数遥控包???星务软件数据接口
//{
//	CPU_INT08U syn[2];     	//同步字
//	//gps
//	CPU_INT16U GpsValid;	//GPS正常标志
//	unsigned int Time;		//北京时间累积秒
//	int WgsPos[3];			//GPS定位位置
//	int WgsSpd[3];			//GPS定位速度
//	unsigned char Sw;		//GPS状态字

//	CPU_INT16U src_alarm;           //能源危机标志
//	//供电
//	unsigned char PwFbSt[MAXSTNUM];    //星敏电压
//	unsigned char PwFbGu[MAXGUNUM];    //陀螺电压
//	unsigned char PwFbRw[4];    		//飞轮电压
//	unsigned char PwFbMm[MAXMMNUM];    //磁强计电压
//	unsigned short PwFbLv;      		//自锁阀电压      
//	unsigned short PwFbMt;      		//磁力矩器电压
//	unsigned short PwFbPp;				//推力器电压
//	//各单机遥测
//	unsigned char StRmBuf[MAXSTNUM][STDATALEN];  //星敏遥测数据
//	unsigned char GuRmBuf[MAXGUNUM][MAXGUDATALEN];  //陀螺遥测数据
//	unsigned char RwXRmBuf[RWDATALEN];          	//X方向飞轮遥测数据
//	unsigned char RwYRmBuf[RWDATALEN];          	//Y方向飞轮遥测数据
//	unsigned char RwZRmBuf[RWDATALEN];          	//Z方向飞轮遥测数据
//	unsigned char RwCRmBuf[RWDATALEN];				//斜装飞轮的遥测数据
//	short MmV[MAXMMNUM][3];                      	//磁强计测量值
//	unsigned char PPRMBuf[PPDATALEN];				//PPCU遥测           
//	short AssCrnt[2][4];                        	//模太电流
//	unsigned char MmValid[MAXMMNUM];				//磁强计有效标志
//	short MtC[3];                      				//磁力矩器输出
//	unsigned char CamType;							//相机类型
//	unsigned char CamDataValid;						//相机数据有效性
//	short CamPnt[2];								//相机像元行列号
//	unsigned char bkBuf[506];
//	CPU_INT08U sum;

//} AOCC_DATA;
#pragma pack()
#pragma pack(1)
typedef struct
{
	CPU_INT08U	mode;	//识别码
	CPU_INT08U	xOut[3];	//X轴角速度
	CPU_INT08U	yOut[3];	//Y轴角速度
	CPU_INT08U	zOut[3];	//Z轴角速度
	CPU_INT08U	fixed;		//固定值
	CPU_INT08U	xTemp[2];	//X轴陀螺温度信号
	CPU_INT08U	yTemp[2];	//Y轴陀螺温度信号
	CPU_INT08U	zTemp[2];	//Z轴陀螺温度信号
	CPU_INT08U	crc;	//校验和
} TL_TM;
#pragma pack()
// 姿控广播数据包
#pragma pack(1)
typedef struct
{
	CPU_INT16U Syn;
	CPU_INT08U Mod;
	CPU_INT08U Len;
	CPU_INT08U AttTime[6];			// 姿态时间
	CPU_INT16U tmp;
	CPU_INT64U ObtTime;				// 轨道时间
	CPU_INT32U AttValid;            // 姿态有效性
	CPU_INT32S ObtAng[3];			// 轨道系123转序角度
	CPU_INT32S ObtW[3];					// 本体系相对轨道坐标系123转序欧拉角变化率
	CPU_INT32S Wib[3];				// 惯性角速度在本体系X轴分量
	//int Qib[4];					// 惯性系姿态四元数
	CPU_INT32U ObtValid;			// 轨道数据有效性
	CPU_INT32S a;					// 轨道半长轴
	CPU_INT32S e;					// 轨道偏心率
	CPU_INT32S i;					// 轨道倾角
	CPU_INT32S omg;					// 升交点赤经
	CPU_INT32S w;					// 近地点幅角
	CPU_INT32S M;					// 平近点角
	CPU_INT32S u;					// 纬度幅角
	CPU_INT32S f;					// 真近点角
	CPU_INT32S orbit_w;				// 轨道角速度（标量）
	//short SunAngle;				// 太阳高度角
	//short SatLati;
	//short SatLong;
	//unsigned int H;					// 地心距
	//unsigned short V;				// 卫星速度V
	//int J2000Pos[3];				// J2000卫星位置
	//int J2000Spd[3];				// J2000卫星速度
	//int WGS84Pos[3];				// WGS84卫星位置
	//int WGS84Spd[3];				// WGS84卫星速度
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





