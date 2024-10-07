/*
 * typedef.h
 *
 *  Created on: 2023年10月13日
 *      Author: linxiaojun
 */

#ifndef SRC_HEAD_TYPEDEF_H_
#define SRC_HEAD_TYPEDEF_H_

#define MAX_PROGTAB_ITEMS 100  // shao

//*********************************************************
//                      头文件
//*********************************************************
#include "define.h"
#include "cpu.h" // shao

//*********************************************************
//                      类型重定义
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
	//注数遥测变量第0 帧
	CPU_INT16U G_CLK                  ; //Z0.1   (01H)软硬计时选择
	CPU_INT16U G_INJK                 ; //Z0.2   (02H)故障启用FLASH标志
	CPU_INT16U G_ALLOWCHECK           ; //Z0.3   (03H)允许起用校验
	CPU_INT16U G_ALLOWPROG            ; //Z0.4   (04H)允许在轨编程标志
	CPU_INT16U Payload_colle_tag      ; //Z0.5   (05H)数据采集标志允许标志
	CPU_INT16U adjust_mode_tag        ; //Z0.6   (06H)系统钟校时选择
	CPU_INT16U gps_limit              ; //Z0.7   (07H)系统钟GPS校时门限
	CPU_INT16U gps_space              ; //Z0.8   (08H)系统钟GPS校时间隔
	CPU_INT16U Syscast_tag            ; //Z0.9   (09H)系统信息广播准禁标志
	CPU_INT16U urgent_wk_limit        ; //Z0.10  (0AH)应急温控门限控制范围
	CPU_INT16U ck_reset_permit        ; //Z0.11  (0BH)程控复位准禁标志
	CPU_INT16U close_permit_tag       ; //Z0.12  (0CH)应急状态标志
	CPU_INT16U PCDU_reset_permit      ; //Z0.13  (0DH)PCDU复位准禁标志
	CPU_INT16U sc_timing_off_permit   ; //Z0.17  (11H)数传定时关机准禁标志
	CPU_INT16U wk_permit_tag          ; //Z0.49  (31H)加热器准禁标志
	CPU_INT16U wk_select_tag          ; //Z0.50  (32H)测温通道主备选择标志
	
	//注数遥测变量第1帧 (复制区)
	CPU_INT16U Hbatt_heat_limit_1     ; //Z1.16   (74H)   电池补偿加热器门限1
	CPU_INT16U Hbatt_heat_limit_2     ; //Z1.17   (75H)   电池补偿加热器门限2
	CPU_INT16U HdownAnt_heat_limit_1  ; //Z1.18   (76H)   GNSS下视低噪放补偿加热器门限1
	CPU_INT16U HdownAnt_heat_limit_2  ; //Z1.19   (77H)   GNSS下视低噪放补偿加热器门限2
	CPU_INT16U HdownAnt_heat_limit_3  ; //Z1.20   (78H)   GNSS下视低噪放补偿加热器门限3
	CPU_INT16U HupAnt_heat_limit      ; //Z1.21   (79H)   GNSS上视低噪放补偿加热器门限
	CPU_INT16U HZH1M_heat_limit       ; //Z1.49   (95H)		一体机主补偿加热器1门限
	CPU_INT16U HZH1B_heat_limit       ; //Z1.50   (96H)		一体机备补偿加热器1门限
	CPU_INT16U HXKZM_heat_limit       ; //Z1.51  (97H)		相控阵主补偿加热器门限
	CPU_INT16U HXKZB_heat_limit       ; //Z1.52   (98H)		相控阵备补偿加热器门限
	CPU_INT16U HZH2M_heat_limit       ; //Z1.53   (99H)		一体机主补偿加热器2门限
	CPU_INT16U batt_protect_switch    ; //Z1.54   (9AH)		过放保护功能标志
	CPU_INT32U CKZJ_TIMING_time       ; //Z1.55-56(9B-9CH)		测控数传1复位时间门限
	CPU_INT16U HZH2B_heat_limit	      ; //Z1.57   (9DH)		一体机备补偿加热器2门限
	CPU_INT16U HXM1_heat_limit	      ; //Z1.58   (9EH)		星敏1补偿加热器门限
	CPU_INT16U HXM2_heat_limit	      ; //Z1.59   (9FH)		星敏2补偿加热器门限
	CPU_INT16U HPCDU_heat_limit       ; //Z1.60   (A0H)		PCDU补偿加热器门限
	
  // 注数遥测变量第1帧 (复制区)

    CPU_INT16U inertialUnit_heat_limit; // 惯组加热器门限
    CPU_INT16U starSensor_heat_limit  ; // 星敏加热器门限
    CPU_INT16U cage_heat_limit        ; // 笼屉加热器门限
    CPU_INT16U coolThrust1_limit      ; // 冷气推力器1加热器门限
    CPU_INT16U coolThrust2_limit      ; // 冷气推力器2加热器门限
    CPU_INT16U coolBottle_limit       ; // 冷气气瓶加热器门限
    CPU_INT16U coolValve_limit        ; // 冷气减压阀加热器门限
    CPU_INT16U catalyticBed1_limit    ; // 单组元催化床1加热器门限
    CPU_INT16U catalyticBed3_limit    ; // 单组元催化床3加热器门限
    CPU_INT16U catalyticBed5_limit    ; // 单组元催化床5加热器门限
    CPU_INT16U catalyticBed6_limit    ; // 单组元催化床6加热器门限
    CPU_INT16U catalyticBed7_limit    ; // 单组元催化床7加热器门限
    CPU_INT16U catalyticBed8_limit    ; // 单组元催化床8加热器门限
    CPU_INT16U fuelTank_limit         ; // 单组元贮箱加热器门限
    CPU_INT16U fuelLine_limit         ; // 单组元管路加热器门限
    CPU_INT16U pressureSensor_limit   ; // 压力传感器加热器门限
    CPU_INT16U solenoidValve1_limit   ; // 电磁阀1加热器门限
    CPU_INT16U solenoidValve2_limit   ; // 电磁阀2加热器门限
    CPU_INT16U observationUnit_limit  ; // 观瞄组件加热器门限
    CPU_INT16U ZDB_limit              ; // ZDB加热器门限
    CPU_INT16U batteryTemp_limit      ; // 蓄电池温度状态加热器门限
		
		// 存储模式下的门限
    CPU_INT16U inertialUnit_storage_limit;  // 惯组加热器存储模式门限
    CPU_INT16U starSensor_storage_limit;    // 星敏加热器存储模式门限
    CPU_INT16U cage_storage_limit;          // 笼屉加热器存储模式门限
    CPU_INT16U coolThrust1_storage_limit;   // 冷气推力器1存储模式门限
    CPU_INT16U coolThrust2_storage_limit;   // 冷气推力器2存储模式门限
    CPU_INT16U coolBottle_storage_limit;    // 冷气气瓶存储模式门限
    CPU_INT16U coolValve_storage_limit;     // 冷气减压阀存储模式门限
    CPU_INT16U catalyticBed1_storage_limit; // 单组元催化床1存储模式门限
    CPU_INT16U catalyticBed3_storage_limit; // 单组元催化床3存储模式门限
    CPU_INT16U catalyticBed5_storage_limit; // 单组元催化床5存储模式门限
    CPU_INT16U catalyticBed6_storage_limit; // 单组元催化床6存储模式门限
    CPU_INT16U catalyticBed7_storage_limit; // 单组元催化床7存储模式门限
    CPU_INT16U catalyticBed8_storage_limit; // 单组元催化床8存储模式门限
    CPU_INT16U fuelTank_storage_limit;      // 单组元贮箱存储模式门限
    CPU_INT16U fuelLine_storage_limit;      // 单组元管路存储模式门限
    CPU_INT16U pressureSensor_storage_limit;// 压力传感器存储模式门限
    CPU_INT16U solenoidValve1_storage_limit;// 电磁阀1存储模式门限
    CPU_INT16U solenoidValve2_storage_limit;// 电磁阀2存储模式门限
    CPU_INT16U observationUnit_storage_limit;// 观瞄组件存储模式门限
    CPU_INT16U ZDB_storage_limit;           // ZDB存储模式门限
    CPU_INT16U batteryTemp_storage_limit;   // 蓄电池温度状态存储模式门限
	
	//注数遥测变量第2 帧 (可编程函数表)

  CPU_INT32U ramtab[MAX_PROGTAB_ITEMS];
	
} ZSTM;

#pragma pack(1)
typedef struct
{
	// 过程遥测变量第0帧
	CPU_INT16U dir_cmd_count         ;  //G0.1     直接指令计数
	CPU_INT16U indir_cmd_count       ;  //G0.2     间接指令计数
	CPU_INT16U inn_cmd_count         ;  //G0.3     内部指令计数
	CPU_INT16U soft_zs_count         ;  //G0.4     软件注数计数
	CPU_INT16U payload_zs_count      ;  //G0.5     业务注数计数
	CPU_INT16U read                  ;  //G0.6     指令队列读指针
	CPU_INT16U write                 ;  //G0.7     指令队列写指针
	CPU_INT16U enter_200ms           ;  //G0.8     200ms入口计数
	CPU_INT16U enter_400ms           ;  //G0.9     400ms入口计数
	CPU_INT16U enter_1s              ;  //G0.10    1s入口计数
	CPU_INT16U enter_2s              ;  //G0.11    2s入口计数
	CPU_INT16U enter_5s              ;  //G0.12    5s入口计数
	CPU_INT16U enter_1m              ;  //G0.13    1分入口计数
	CPU_INT16U prog_state            ;  //G0.14    程序运行状态
	CPU_INT16U comp_rule_state       ;  //G0.15    计算机状态标志
	CPU_INT16U yc_status3            ;  //G0.28	  遥控状态遥测3
	CPU_INT16U syn_ok_cnt            ;  //G0.29   同步头符合计数
	CPU_INT16U heater_state          ;  //G0.36    加热器状态标志
	CPU_INT16U vcid_ok_cnt           ;  //G0.43VCID符合计数
	CPU_INT32U ckzj_timing_count     ;  //G0.69_70  中继测控定时计数
	CPU_INT16U frame_ok_cnt          ;  //G0.71 帧格式符合计数
	CPU_INT16U first_power_tag       ;  //G0.85首次上电标志；
	
	// 过程遥测变量第2帧(从注数遥测第2帧复制）//

	CPU_INT32U TeYK                    ; //G1.1_2      系统钟均匀校时量
	CPU_INT32U GDZeYk                  ; //G1.3_4      轨道钟均匀校时量
	CPU_INT32U SoftTime_S              ; //G1.5_6      系统钟时间秒计数
	CPU_INT16U SoftTime_mS             ; //G1.7        系统钟时间亚秒计数(ms)
	CPU_INT16U OrbiCount               ; //G1.8        轨道圈数
	CPU_INT32U GDZs                    ; //G1.9_10     轨道钟授时量(ms)
	CPU_INT32S DeltaGDZ                ; //G1.11_12    轨道钟的集中校时量
	CPU_INT32S Delta_SysTime_s         ; //G1.13_14    系统钟集中校时量秒计数
	CPU_INT16S Delta_SysTime_ms        ; //G1.15      系统钟集中校时量毫秒计数
	CPU_INT16U Tm_baud                 ; //G1.24   (7CH)   遥测码速率切换20170904修改
	CPU_INT16U Tm_down_tag             ; //G1.29   (81H)   遥测下传方式字
	CPU_INT16U autowk_permit_tag       ; //G1.39      自动温控允许标志
	CPU_INT16U Zkfault                 ; //G1.40      wxy蓄电池组过放电标志。共3阶段
	CPU_INT16U first_order_Vth         ; //G1.42    过放电保护流程允许电压设置V1(一阶段蓄电池电压阈值)(10.8V)
	CPU_INT16U second_order_Vth        ; //G1.43    过放电保护蓄电池组电压设置V2(二阶段蓄电池电压阈值)(10.5V)
	CPU_INT16U third_order_Vth        ; //G1.44    过放电保护蓄电池组电压设置V3(三阶段蓄电池电压阈值)(10.2V)
	CPU_INT16U batts_Vth               ; //G1.45    蓄电池组电压阈值(10.8V)
	CPU_INT16U batt_protect_switch    ; //G1.54   (9AH)		过放保护功能标志
	CPU_INT16U energy_management_tag   ; //G1.63	能源管理准禁
	CPU_INT16U auto_connect_enable_state; //G1.64	放电开关自主接通使能标志位
	CPU_INT16U current_out_Vth         ; //G1.66   过放电保护蓄电池放电电流设置V8(0.5A)
	CPU_INT16U batts_temp_Vth          ; //G1.67   放电开关自主接通蓄电池温度阈值(0度)
} GCTM;
#pragma pack()

// shao 定义指令队列
#pragma pack(1)
typedef struct					//定义指令队列
{
    uint32_t addr_on;   // 地址（32位）
    uint16_t cmd_on;    // 开命令（16位）
    uint32_t addr_off;  // 地址（32位）
    uint16_t cmd_off;   // 关命令（16位）
}CMD_Q;
#pragma pack()

/*组件状态*/
typedef struct ZJ_STATE
{
	uint8_t			ucGMCan;						//有效GM Can
	uint8_t			ucTXJCan;						//有效TXJ Can
	uint8_t			ucWX_YCMode;					//无线遥测模式
	uint8_t			ucSecondFlag;					//秒标记
	uint8_t			ucSecondCnt;					//秒计数
	Uint32			uiPingPang;						//乒乓
	Uint32			auiImgSendDataReady[2];			//降采样图像数据准备好标记，[0]分离前，[1]分离后
	Uint8			ucImageReturnState;				//图像回传状态
	Uint8			ucSZProgramCRCState;			//观瞄处理组件上注程序校验状态
	Uint8			ucProgramSXState;				//观瞄处理组件程序烧写完成状态
	Uint32			uiZDBAttackTime;				//zdb打击时刻
	Uint8			ucCameraReloadFlag;				//相机重载标志
	Uint8			ucCameraBootFlag;				//相机启动标志，0,表示未启动（未发送配置参数），1表示相机正常启动，2表示相机启动后3s使能FPGA图传，3表示相机正常返遥测
	Uint8			ucDEMBootFlag;					//测距机启动标志，0表示未上电且未授时，1表示测距机上电且未授时，2表示测距机上电且授时，3表示测距机正常返遥测
	Uint8			ucSpacecraftIDFlag;				//航天器识别码标志，0表示没有航天器识别码，1表示有航天器识别码
	Uint32			uiSecondCnt;					//心跳（秒计数）
	Uint8			ucXMRecvFlag;					//星敏数据接收判断
	Uint8			ucGNSSNewFlag;					//GNSS数据接收判断
	Uint8			ucZSSFlag;						//自守时标志
	Uint32			uiUTCSecond;					//UTC时间，秒
	Uint32			uiUTCMicroSecond;				//UTC时间，微秒
	Uint8			ucBCEnable;						//广播使能标记
	Uint8			ucZDBBootFlag;					//战斗部启动标志
	Uint32			uiZDBSelfCheckTime;				//ZDB自检时刻ms
	Uint32			uiSelfCheckTime;				//自检时刻s
	Uint8			ucSelfCheckFlag;				//自检标记
	Uint8			ucSelfCheckState;				//自检状态
	Uint32			uiGPSUTCSecond;
	Uint32			uiGPSUTCMicroSecond;
	float			fLat;
	float			fLon;
	float			fHeight;
}T_ZJ_STATE;

/*核1共享给核0的算法运行结果状态*/
typedef struct ALG_STATE
{
	Uint8			ucImgGrayAvg;
	Uint8			ucImgGrayStd;
		
	Uint8			ucGMSoftWareMode;
		
	Uint8			ucRegionNumState;
	Uint8			ucRegionNum:4;					//B7~B4：二值化后的连通域数量
	Uint8			ucDelTargetNum:4;				//B3~B0：剔除虚警后目标数量
		
	Uint8			ucImgTargetCaptureState;
	Uint8			ucCameraOverexposure:2;			//B7-B6：相机过曝状态
	Uint8			uc1QuadrantStrongLight:1;		//B5：第一象限强光标志
	Uint8			uc2QuadrantStrongLight:1;		//B4：第二象限强光标志
	Uint8			uc3QuadrantStrongLight:1;		//B3：第三象限强光标志
	Uint8			uc4QuadrantStrongLight:1;		//B2：第四象限强光标志
	Uint8			ucTargetCaptureMode:1;			//B1：目标捕获方式
	Uint8			ucTargetCaptureFlag:1;			//B0：目标捕获标志

	Uint8			ucStarState;
	Uint8			ucTriangularMatchFlag:4;		//三角匹配判断
	Uint8			ucXMDataRecvFlag:2;				//星敏数据接收判断
	Uint8			ucStarDelFlag:2;				//恒星剔除判断

	float			fRA;							//相机视轴指向天区赤经输出
	float			fDEC;							//相机视轴指向天区赤纬输出
	Uint8			ucTargetFlightFlag;				//目标轨迹关联判断
	Uint8			ucTargetPosFlag;				//目标位置判断
	Uint16			usTargetBright;					//目标亮度输出
	Uint16			usTargetShape;					//目标形状输出
	Uint32			uiTargetSize;					//目标面积输出
	Uint8			ucTargetConfidence;				//目标置信度输出
	float			fTargetAZ;						//目标投影角α的tan值输出
	float			fTargetEL;						//目标投影角β的tan值输出
}T_ALG_STATE;

/*核1共享给核0的位姿解算信息*/
typedef struct GESTURE_INFO
{
	/*用于位姿解算的信息*/
	Uint32			*puiBinImg;						//待位姿解算二值化图像地址
	Uint16			usXCentroid;					//目标X方向坐标
	Uint16			usYCentroid;					//目标Y方向坐标
	Uint8			ucGestureFlag;					//位姿计算标记
}T_GESTURE_INFO;

/*核1共享给核0的降采样图像信息*/
typedef struct DOWN_IMG_INFO
{
	Uint8			ucDownImgType;					//降采样图像类型，1：分离前，2：分离后
}T_DOWN_IMG_INFO;

/*核1共享给核0的信息*/
typedef struct CORE1TO0_SHARE_INFO
{
	/*状态类型*/
	Uint8			ucType;							//0x11：只有算法运行结果状态，0x22：包含位姿，0x33：包含降采样图像信息，0x44：都包含

	/*算法运行结果状态*/
	T_ALG_STATE 	tAlgState;

	/*位姿信息*/
	T_GESTURE_INFO 	tGestureInfo;

	/*降采样图像信息*/
	T_DOWN_IMG_INFO	tDownImgInfo;
}T_CORE1TO0_SHARE_INFO;

/*核0共享给核1的信息*/
typedef struct CORE0TO1_SHARE_INFO
{
	Uint8			ucType;							//共享信息类型
	/*指令*/
	Uint8			ucImageReturnApply;				//图像回传申请指令				0x11
	Uint8			ucGMMode;						//观瞄软件模式(不同形状目标)	0x12
	Uint8			ucImgReverse;					//图像反转						0x13
	/*通知核1新的星敏数据*/
	float			fquatenion0;					//四元数0   					0x22
	float			fquatenion1;					//四元数1
	float			fquatenion2;					//四元数2
	float			fquatenion3;					//四元数3
	/*通知核1的分离后图像采集标记*/
	Uint8			aucImageStoreFlag[4];			//分离后图像采集标志			0x33

}T_CORE0TO1_SHARE_INFO;

/*角点连通信息*/
typedef struct CORNER_LINK
{
	Uint16 			usXOrd;							//角点x方向坐标
	Uint16 			usYOrd;							//角点y方向坐标
	Uint16			usLinkCnt8;						//角点8连通点数
	Uint16			usUnLinkCnt24;					//角点24不连通点数
}T_CORNER_LINK;

/*连通域信息*/
typedef struct REGION_INFO
{
	Uint32 			uiRegionSize;					//连通域大小
	Uint16			usEdgeLen;						//边缘长度
	Uint16			usCornerLen;					//总角点数
	T_CORNER_LINK 	*ptCornerLink;					//角点连接信息
}T_REGION_INFO;

/*核0位姿解算状态*/
typedef struct GESTURE_STATE
{
	Uint8			ucTargetDisState;				//目标软测距状态
	Uint16			usTargetDis;					//目标软测距距离1
	Uint8			ucTargetDis;					//目标软测距距离2
	float			fTargetXGesture;				//目标X轴姿态
	float			fTargetYGesture;				//目标Y轴姿态
	float			fTargetZGesture;				//目标Z轴姿态
}T_GESTURE_STATE;

/*角点信息*/
typedef struct CORNER_INFO
{
	Uint16 			usXOrd;							//角点x方向坐标
	Uint16 			usYOrd;							//角点y方向坐标
	Uint16			usDescripNum;					//角点描述子总数
	Uint8			*pucDescrip;					//角点描述子
}T_CORNER_INFO;

/*匹配特征点的2D信息*/
typedef struct POINT_2D
{
	float 			fPointXord;						//x方向坐标
	float 			fPointYord;						//y方向坐标
	Uint8			ucFlag;							//特征点选择标记
}T_POINT_2D;

/*匹配特征点的3D信息*/
typedef struct POINT_3D
{
	float 			fPointXord;						//x方向坐标
	float 			fPointYord;						//y方向坐标
	float 			fPointZord;						//z方向坐标
}T_POINT_3D;

/*复数*/
typedef struct COMPLEX
{
	double 			dR;								//实部
	double 			dI;								//虚部
}T_COMPLEX;



#endif /* SRC_HEAD_TYPEDEF_H_ */
