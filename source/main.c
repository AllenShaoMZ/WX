#include "math.h"
#include "mss_assert.h"
#include "sys_config.h"

#include "mss_uart.h"
#include "mss_gpio.h"
#include "mss_timer.h"

#include "FPGAUart.h"
#include "FPGACan.h"
#include "can.h"
#include "float.h"
#include "os.h"        

#include "bsp.h"
#include "includes.h"
#include "m2sxxx.h"

#include "main.h"
#include "time.h"
#include "typedef.h"
#include "flyprog.h"
#include "memory.h" 

#include "extern.h"

// 设置堆栈空间
#define APP_CFG_TASK_START_STK_SIZE  512
#define MAIN_TASK_STK_SIZE           512
#define YK_TASK_STK_SIZE             512
#define YC_TASK_STK_SIZE             512
#define CAN_TASK_STK_SIZE            512
#define AOCC_TASK_STK_SIZE           512
#define FLY_TASK_STK_SIZE            512

// 设置优先级
#define APP_CFG_TASK_START_PRIO      5
#define MAIN_TASK_PRIO               14
#define YK_TASK_PRIO                 12
#define YC_TASK_PRIO                 11
#define CAN_TASK_PRIO                10
#define AOCC_TASK_PRIO               9
#define FLY_TASK_PRIO                8

static OS_TCB    AppTaskStartTCB;
static OS_TCB    MainTaskTCB;
static OS_TCB    YkTaskTCB;
static OS_TCB    YCTaskTCB;
static OS_TCB    CanTaskTCB;
static OS_TCB    AOCCTaskTCB;
static OS_TCB    FLYTaskTCB;

static CPU_STK   AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK   MainTaskStk[MAIN_TASK_STK_SIZE];
static CPU_STK   YkTaskStk[YK_TASK_STK_SIZE];
static CPU_STK   YCTaskStk[YC_TASK_STK_SIZE];
static CPU_STK   CanTaskStk[CAN_TASK_STK_SIZE];
static CPU_STK   AOCCTaskStk[YC_TASK_STK_SIZE];
static CPU_STK   FLYTaskStk[CAN_TASK_STK_SIZE];

static int cycle_counter = 0;  // 用于控制任务的周期性执行，设为全局变量

GCTM GC; // 修改
ZSTM ZS;

CPU_INT08U G_TIMETA = 0;

// 外部声明 SystemCoreClock 变量
extern uint32_t SystemCoreClock;

int main(void)
{
	
		OS_ERR err;
		CPU_INT32U systick_clk;
		/*可以放在系统硬件初始化*/
		CPU_Init(); /* Initialize the uC/CPU services */
		CPU_IntDis();	 

		/*系统硬件初始化*/
		System_HW_Init();
		
		/*系统软件初始化*/
		System_SW_Init();
		
		FPGAUart_SendString(UART_CX_TXDATA_ADDR_DEBUG, "---arm CX---\r\n" );
		FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "---arm DC--\r\n" ); 
		FPGAUart_SendString(UART_TXJ_TXDATA_ADDR_DEBUG, "---arm TXJ---\r\n" );
		FPGAUart_SendString(UART_GZ_TXDATA_ADDR_DEBUG, "---arm GZ--\r\n" );	

		OSInit(&err);  /* Init uC/OS-III */
	
		/* 初始化滴答定时器 */
		systick_clk = SystemCoreClock / 1000; // 每1ms产生一个滴答中断
		SysTick_Init(systick_clk);

		/* 创建 App Task Start */
		OSTaskCreate((OS_TCB *)&AppTaskStartTCB,
								 (CPU_CHAR *)"App Task start",
								 (OS_TASK_PTR)AppTaskStart,
								 (void *)0,
								 (OS_PRIO)APP_CFG_TASK_START_PRIO,
								 (CPU_STK *)&AppTaskStartStk[0],
								 (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE / 10,
								 (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE,
								 (OS_MSG_QTY)0u,
								 (OS_TICK)0u,
								 (void *)0,
								 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								 (OS_ERR *)&err);

		OSStart(&err);  // 启动操作系统
}

void  Init_vari(void)
{
    CPU_INT32U i = 0;
    CPU_INT64U * pData =(CPU_INT64U * )0x64000000;//保留区数据段首地址--找一个地址
    CPU_INT64U * pData1=(CPU_INT64U * )0x64200000;
    CPU_INT64U * pData2=(CPU_INT64U * )0x64400000;	
   
		SaveTo3_16(&(ZS.PCDU_reset_permit),0);		//PCDU设备管理默认禁止
		SaveTo3_16(&(GC.energy_management_tag),0xaaaa);	//能源管理默认禁止
		SaveTo3_16(&(GC.write),0);
		SaveTo3_16(&(GC.read),0);
		SaveTo3_16(&(GC.Tm_down_tag),0x5555);/*遥测初值化为实时模式*/
		SaveTo3_16(&GC.autowk_permit_tag,0x0);			//20180528自动温控允许标志，初始化为禁止
		SaveTo3_32(&GC.GDZs,0x550A0); //348320ms
		SaveTo3_16(&GC.OrbiCount,1);
		SaveTo3_32(&GC.SoftTime_S,0x2A4FA658);		//2022-6-30 12:10:0   
		SaveTo3_16(&GC.SoftTime_mS,0);
		SaveTo3_16(&GC.first_order_Vth,1080);			//wxy:10.8V
		SaveTo3_16(&GC.second_order_Vth,1050);			//wxy:10.5V
		SaveTo3_16(&GC.third_order_Vth,1020);			//wxy:10.2V
		SaveTo3_16(&GC.batts_Vth,1050);				//10.5V
		SaveTo3_16(&GC.current_out_Vth,50);			//过放电保护蓄电池放电电流阈值。初始化为0.5A
		SaveTo3_16(&GC.batts_temp_Vth,0x0056);				//放电开关自主接通蓄电池温度阈值。初始化为0度
		SaveTo3_16(&GC.Zkfault,0x0);					//20180528蓄电池组过放标志，初始化为正常
		SaveTo3_16(&(GC.batt_protect_switch),0x0);		//过放保护功能准禁.初始化为禁止
		SaveTo3_16(&ZS.G_INJK,0x5555);//故障启用ROM
		SaveTo3_16(&ZS.G_CLK,0x146f);
		SaveTo3_16(&ZS.gps_limit,0x14);
		SaveTo3_16(&ZS.gps_space,0x258);
		SaveTo3_16(&ZS.adjust_mode_tag,0x5555);//默认为GPS校时
		SaveTo3_16(&ZS.close_permit_tag,0x5555);
		SaveTo3_16(&ZS.wk_select_tag,0xFFFF);
		SaveTo3_16(&ZS.wk_permit_tag,0);//20180130允许蓄电池、//允许推力系统加热器 初始值应为0	
		SaveTo3_16(&ZS.urgent_wk_limit,0x10);
		//温度	0x5E -8du | 0x5B -6du | 0x3C 13du | 0x39 15du | 0x36 17du
		SaveTo3_16(&ZS.Hbatt_heat_limit_1,0x3639);		//[15,17]	[15.01,16.98]
		SaveTo3_16(&ZS.Hbatt_heat_limit_2,0x393C);		//[13,15]	[13.10,15.02]
		SaveTo3_16(&ZS.HdownAnt_heat_limit_3,0x5B5E);	//[-8,-6]	[-8.34,-6.28]
		SaveTo3_16(&ZS.HupAnt_heat_limit,0x5B5E);		//[-8,-6]	[-8.34,-6.28]
		SaveTo3_16(&ZS.HdownAnt_heat_limit_1,0x5B5E);	//[-8,-6]	[-8.34,-6.28]
		SaveTo3_16(&ZS.HdownAnt_heat_limit_2,0x5B5E);	//[-8,-6]	[-8.34,-6.28]
		
		// 温度范围设定：将各个加热器门限设置在15到20度之间
	// 0x3C -> 15度 | 0x3F -> 16度 | 0x42 -> 17度 | 0x45 -> 18度 | 0x48 -> 19度 | 0x4B -> 20度
		SaveTo3_16(&ZS.inertialUnit_heat_limit, 0x3C4B);   // 惯组加热器门限：[15,20]
		SaveTo3_16(&ZS.starSensor_heat_limit, 0x3C4B);     // 星敏加热器门限：[15,20]
		SaveTo3_16(&ZS.cage_heat_limit, 0x3C4B);           // 笼屉加热器门限：[15,20]
		SaveTo3_16(&ZS.coolThrust1_limit, 0x3C4B);         // 冷气推力器1加热器门限：[15,20]
		SaveTo3_16(&ZS.coolThrust2_limit, 0x3C4B);         // 冷气推力器2加热器门限：[15,20]
		SaveTo3_16(&ZS.coolBottle_limit, 0x3C4B);          // 冷气气瓶加热器门限：[15,20]
		SaveTo3_16(&ZS.coolValve_limit, 0x3C4B);           // 冷气减压阀加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed1_limit, 0x3C4B);       // 单组元催化床1加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed3_limit, 0x3C4B);       // 单组元催化床3加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed5_limit, 0x3C4B);       // 单组元催化床5加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed6_limit, 0x3C4B);       // 单组元催化床6加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed7_limit, 0x3C4B);       // 单组元催化床7加热器门限：[15,20]
		SaveTo3_16(&ZS.catalyticBed8_limit, 0x3C4B);       // 单组元催化床8加热器门限：[15,20]
		SaveTo3_16(&ZS.fuelTank_limit, 0x3C4B);            // 单组元贮箱加热器门限：[15,20]
		SaveTo3_16(&ZS.fuelLine_limit, 0x3C4B);            // 单组元管路加热器门限：[15,20]
		SaveTo3_16(&ZS.pressureSensor_limit, 0x3C4B);      // 压力传感器加热器门限：[15,20]
		SaveTo3_16(&ZS.solenoidValve1_limit, 0x3C4B);      // 电磁阀1加热器门限：[15,20]
		SaveTo3_16(&ZS.solenoidValve2_limit, 0x3C4B);      // 电磁阀2加热器门限：[15,20]
		SaveTo3_16(&ZS.observationUnit_limit, 0x3C4B);     // 观瞄组件加热器门限：[15,20]
		SaveTo3_16(&ZS.ZDB_limit, 0x3C4B);                 // ZDB加热器门限：[15,20]
		SaveTo3_16(&ZS.batteryTemp_limit, 0x3C4B);         // 蓄电池温度状态加热器门限：[15,20]
		
		// 存储模式的门限
		SaveTo3_16(&ZS.inertialUnit_storage_limit, 0x3A49);  // 惯组加热器存储模式门限：[14,19]
		SaveTo3_16(&ZS.starSensor_storage_limit, 0x3A49);    // 星敏加热器存储模式门限：[14,19]
		SaveTo3_16(&ZS.cage_storage_limit, 0x3A49);          // 笼屉加热器存储模式门限：[14,19]
		SaveTo3_16(&ZS.coolThrust1_storage_limit, 0x3A49);   // 冷气推力器1存储模式门限：[14,19]
		SaveTo3_16(&ZS.coolThrust2_storage_limit, 0x3A49);   // 冷气推力器2存储模式门限：[14,19]
		SaveTo3_16(&ZS.coolBottle_storage_limit, 0x3A49);    // 冷气气瓶存储模式门限：[14,19]
		SaveTo3_16(&ZS.coolValve_storage_limit, 0x3A49);     // 冷气减压阀存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed1_storage_limit, 0x3A49); // 单组元催化床1存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed3_storage_limit, 0x3A49); // 单组元催化床3存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed5_storage_limit, 0x3A49); // 单组元催化床5存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed6_storage_limit, 0x3A49); // 单组元催化床6存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed7_storage_limit, 0x3A49); // 单组元催化床7存储模式门限：[14,19]
		SaveTo3_16(&ZS.catalyticBed8_storage_limit, 0x3A49); // 单组元催化床8存储模式门限：[14,19]
		SaveTo3_16(&ZS.fuelTank_storage_limit, 0x3A49);      // 单组元贮箱存储模式门限：[14,19]
		SaveTo3_16(&ZS.fuelLine_storage_limit, 0x3A49);      // 单组元管路存储模式门限：[14,19]
		SaveTo3_16(&ZS.pressureSensor_storage_limit, 0x3A49);// 压力传感器存储模式门限：[14,19]
		SaveTo3_16(&ZS.solenoidValve1_storage_limit, 0x3A49);// 电磁阀1存储模式门限：[14,19]
		SaveTo3_16(&ZS.solenoidValve2_storage_limit, 0x3A49);// 电磁阀2存储模式门限：[14,19]
		SaveTo3_16(&ZS.observationUnit_storage_limit, 0x3A49);// 观瞄组件存储模式门限：[14,19]
		SaveTo3_16(&ZS.ZDB_storage_limit, 0x3A49);            // ZDB存储模式门限：[14,19]
		SaveTo3_16(&ZS.batteryTemp_storage_limit, 0x3A49);    // 蓄电池温度状态存储模式门限：[14,19]
}

/* App Task Start */
void AppTaskStart(void *p_arg)
{
    OS_ERR err;
    char errMsg[50];

    /* 创建 Main Task */
    OSTaskCreate((OS_TCB *)&MainTaskTCB,
                 (CPU_CHAR *)"Main Task",
                 (OS_TASK_PTR)MainTask,
                 (void *)0,
                 (OS_PRIO)MAIN_TASK_PRIO,
                 (CPU_STK *)&MainTaskStk[0],
                 (CPU_STK_SIZE)MAIN_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)MAIN_TASK_STK_SIZE,
                 (OS_MSG_QTY)5u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "MainTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }

    /* 创建 Yk Task */
    OSTaskCreate((OS_TCB *)&YkTaskTCB,
                 (CPU_CHAR *)"Yk Task",
                 (OS_TASK_PTR)YkTask,
                 (void *)0,
                 (OS_PRIO)YK_TASK_PRIO,
                 (CPU_STK *)&YkTaskStk[0],
                 (CPU_STK_SIZE)YK_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)YK_TASK_STK_SIZE,
                 (OS_MSG_QTY)0u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "YkTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }
		
		
			    /* 创建 YC Task */
    OSTaskCreate((OS_TCB *)&YCTaskTCB,
                 (CPU_CHAR *)"YC Task",
                 (OS_TASK_PTR)YCTask,
                 (void *)0,
                 (OS_PRIO)YC_TASK_PRIO,
                 (CPU_STK *)&YCTaskStk[0],
                 (CPU_STK_SIZE)YC_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)YC_TASK_STK_SIZE,
                 (OS_MSG_QTY)0u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "YCTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }

    /* 创建 Can Task */
    OSTaskCreate((OS_TCB *)&CanTaskTCB,
                 (CPU_CHAR *)"Can Task",
                 (OS_TASK_PTR)CanTask,
                 (void *)0,
                 (OS_PRIO)CAN_TASK_PRIO,
                 (CPU_STK *)&CanTaskStk[0],
                 (CPU_STK_SIZE)CAN_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)CAN_TASK_STK_SIZE,
                 (OS_MSG_QTY)0u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "CanTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }
		/* 创建 AOCC Task */
    OSTaskCreate((OS_TCB *)&AOCCTaskTCB,
                 (CPU_CHAR *)"AOCC Task",
                 (OS_TASK_PTR)AOCCTask,
                 (void *)0,
                 (OS_PRIO)AOCC_TASK_PRIO,
                 (CPU_STK *)&AOCCTaskStk[0],
                 (CPU_STK_SIZE)AOCC_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)AOCC_TASK_STK_SIZE,
                 (OS_MSG_QTY)0u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "AOCCTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }
		/* 创建 FLY Task */
    OSTaskCreate((OS_TCB *)&FLYTaskTCB,
                 (CPU_CHAR *)"FLY Task",
                 (OS_TASK_PTR)FLYTask,
                 (void *)0,
                 (OS_PRIO)FLY_TASK_PRIO,
                 (CPU_STK *)&FLYTaskStk[0],
                 (CPU_STK_SIZE)FLY_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE)FLY_TASK_STK_SIZE,
                 (OS_MSG_QTY)0u,
                 (OS_TICK)0u,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    if (err != OS_ERR_NONE) {
        sprintf(errMsg, "FLYTask creation error: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }
		
		OSTaskDel((OS_TCB  *) &AppTaskStartTCB,
               (OS_ERR  *) &err);
		if (err != OS_ERR_NONE) {
        sprintf(errMsg, "Failed to delete AppTaskStart: %d\r\n", err);
        FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, errMsg);
    }
}

/* Main Task */
void MainTask(void *p_arg)
{
		OS_ERR err;
		OS_TICK task_start_clk=0,task_end_clk=0;
//		static int task_counter = 0;  // 用于控制 CanLX_send 的调用频率
		(void)p_arg;
		
		while (1)
		{
			task_start_clk=OSTimeGet(&err);	
			ARMDOG_EN_ADDR = 0x5555;  // 喂狗
			
			CanLX_send();  // 依次发送 GMYCLX、TXJYCLX、GMYKCMD、TXJYKCMD		
			systime_add();
			
			ck_main_prog();
			OSTimeDly(task_start_clk+20,OS_OPT_TIME_MATCH,&err);			
//      cmdDequeue();
			OSTimeDly(task_start_clk+22,OS_OPT_TIME_MATCH,&err);			

			OSTimeDly(task_start_clk+40,OS_OPT_TIME_MATCH,&err);			
			XM_Task();		
	//			AOCC_Task();
			task_end_clk =OSTimeGet(&err);
			G_TIMETA =(CPU_INT08U)(task_end_clk - task_start_clk);
			
			OSTimeDly(100,OS_OPT_TIME_PERIODIC,&err);	 //周期延时100ms

		}
}

/* Yk Task */
void YkTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "YkTask running...\r\n");
    while (1)
    {
			  // Yk任务处理逻辑
        CX_UART_Data_Analysis(eCX);
		   /*DC串口数据解析*/
		    DC_UART_Data_Analysis(eDC);
			
			
        TXJ_UART_Data_Analysis(eDC);
        
        OSTimeDly(10, OS_OPT_TIME_PERIODIC, &err);  // 周期延时10ms
    }
}

/* YC Task */
void YCTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "YCTask running...\r\n");
	
    while (1)
    {
						/*无线遥测组包*/
				TXJ_UART_YC_Pack();
				
				/*无线遥测发送*/
				TXJ_UART_YC_Send();
			
//			  CAN_Data_Send();
        
        OSTimeDly(10, OS_OPT_TIME_PERIODIC, &err);  // 周期延时10ms
    }
}

/* Can Task */
void CanTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "CanTask running...\r\n");
	
    while (1)
    {
				CAN_Data_Analysis();  // 分析接收到的CAN数据
        OSTimeDly(10, OS_OPT_TIME_PERIODIC, &err);  // 周期延时10ms
    }
}
/* AOCC Task */
void AOCCTask(void *p_arg)
{
    (void)p_arg;
    
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "AOCCTask running...\r\n");
	
}
/* FLY Task */
void FLYTask(void *p_arg)
{
    (void)p_arg;
    
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "FLYTask running...\r\n");

}

void CanLX_send(void)
{
    OS_ERR err;
    char buffer[100];
	  unsigned long current_time;
	
	  current_time = OSTimeGet(&err);
      
    if (cycle_counter < 4)
    {     
			switch (cycle_counter) {
        case 0:
					g_tCanSendState[0].ucSendFlag = 1;
				  g_tCanSendState[0].tGMYCLX.uiFrameNum = 1;
					
					sprintf(buffer, "GMYCLX send: time: %lu ticks\r\n", current_time);
					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, buffer);
					break;
				case 1:
					g_tCanSendState[0].ucSendFlag = 1;
					g_tCanSendState[0].tTXJYCLX.uiFrameNum = 1;
					
					sprintf(buffer, "TXJYCLX send, time: %lu ticks\r\n", current_time);
					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, buffer);
					break;
				case 2:
					g_tCanSendState[0].ucSendFlag = 1;
					g_tCanSendState[0].tGMYKCMD.uiFrameNum = 1;
					
					sprintf(buffer, "GMYKCMD send, time: %lu ticks\r\n", current_time);
					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, buffer);
					break;
				case 3:
					g_tCanSendState[0].ucSendFlag = 1;
					g_tCanSendState[0].tTXJYKCMD.uiFrameNum = 1;
					
					sprintf(buffer, "TXJYKCMD send, time: %lu ticks\r\n", current_time);
					FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, buffer);
					break;
                default:
                    break;
            }
			CAN_Data_Send();					
            cycle_counter = (cycle_counter + 1) % 4;						
     }    
}

/* 姿轨控Task */
void XM_Task(void)
{
    OS_ERR err;  
	// 此处编写其他任务逻辑
    FPGAUart_SendString(UART_DC_TXDATA_ADDR_DEBUG, "XM_Task running...\r\n");
    // 模拟任务执行
    OSTimeDly(20, OS_OPT_TIME_PERIODIC, &err);  // 延时60ms
}

void prep_oper(void)
{
	entry_manage();
	gps_ok_proce();
	ckstate_update();
	
}
void ck_main_prog (void)//0911修改?是否是0.5s区
{
	CPU_INT16U GC_enter_1s=0;
	CPU_INT16U GC_enter_1m=0;
	CPU_INT16U ZS_adjust_mode_tag=0;
	ZS_adjust_mode_tag = getVarIn3_16(&ZS.adjust_mode_tag);
	GC_enter_1s = getVarIn3_16(&GC.enter_1s);
	GC_enter_1m = getVarIn3_16(&GC.enter_1m);
	prep_oper();

	if (GC_enter_1s == 3)
		{
			if (ZS_adjust_mode_tag == 0x5555)
				{

				}
			else
				{
					
				}
		}

	switch(GC_enter_1s)
		{
		case 0:

			break;
		case 1:
			temperature_control();
			break;
		case 2:
			(*func_ck_reset)();
			break;

		default:
			break;
		}

}

/* SysTick 初始化 */
void SysTick_Init(uint32_t ticks)
{
    // 配置 SysTick 计数值
    SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;
    // 设置优先级
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    // 清除当前计数值
    SysTick->VAL   = 0;
    // 启用 SysTick：时钟源为内核时钟，启用定时器和中断
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;
}

/* SysTick Handler */
void SysTick_Handler(void)
{
    OSIntEnter();    // 通知 uC/OS-III 进入中断
    OSTimeTick();    // 更新系统时间
    OSIntExit();     // 通知 uC/OS-III 退出中断
}
