#include "extern.h"
#include "mss_gpio.h"
#include "mss_timer.h"
#include "FPGAUart.h"
#include "FPGACan.h" 


//*********************************************************
//                      全局变量定义
//*********************************************************
T_ZJ_STATE g_ZJState;


/************************************************************************
    函  数  名: System_HW_Init
    函数说明: 系统硬件初始化
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void System_HW_Init(void)
{
	SYSREG->WDOG_CR = 0;			      	/* Turn off the watchdog */
	
	ARMDOG_EN_ADDR	= 0x5555;
	
	FPGA_REG_RW_2BYTE(PPS_EN_DISABLE_ADDR) = 0xAAAA;//使能外部秒脉冲
		
	/*FPGA串口初始化*/
	FPGAUart_Init();
	
	/*GPIO初始化*/
	GPIO_init();
	
	/*M3CAN初始化*/
//	CAN_Init();
//	SJACanInit_PELI(CAN_A_ADDRESS); 
	CANIP_Init(0);
	CANIP_Init(1);
	
	/*定时器初始化*/
//	Time1_init();
}

/************************************************************************
    函  数  名: System_SW_Init
    函数说明: 系统软件初始化
    输入参数: 无
    输出参数: 无
    修改说明:
    修  改  人: 林晓俊
************************************************************************/
void System_SW_Init(void)
{
	g_ZJState.ucGMCan = 0;
	g_ZJState.ucTXJCan = 1;
	g_ZJState.ucWX_YCMode = 0;//重要遥测下传模式
//	g_ZJState.ucWX_YCMode = 1;//工程遥测下传模式
	
	
	/*串口相关变量初始化*/
	UART_Soft_Init();
	
	/*CAN相关变量初始化*/
	CAN_Soft_Init();
}

/*Time1初始化*/
//void Time1_init(void)
//{
//	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE);
//    MSS_TIM1_load_immediate(16000000u);
//    MSS_TIM1_start();
//    MSS_TIM1_enable_irq();
//}

/*定时器中断服务函数*/
void Timer1_IRQHandler(void)
{
	static uint32_t uiSecond = 0;
	
	uiSecond = uiSecond + 1;
	
	g_ZJState.ucSecondFlag = 1;
	
	g_ZJState.ucSecondCnt = g_ZJState.ucSecondCnt + 1;
	
	/*每秒向观瞄发送遥测轮询*/
	g_tCanSendState[g_ZJState.ucGMCan].tGMYCLX.uiFrameNum = 1;

	/*每秒向通信机发送遥测轮询*/
	g_tCanSendState[g_ZJState.ucTXJCan].tTXJYCLX.uiFrameNum = 1; 

    /*清除定时器中断标志位*/
	MSS_TIM1_clear_irq(); 
}

/*GPIO初始化*/
void GPIO_init(void)
{
	MSS_GPIO_init();

	/*CANA中断*/
	MSS_GPIO_config(MSS_GPIO_9 , MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW);
	MSS_GPIO_enable_irq(MSS_GPIO_9);	
	
	/*CANB中断*/
	MSS_GPIO_config(MSS_GPIO_10 , MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW);
	MSS_GPIO_enable_irq(MSS_GPIO_10);
	
	MSS_GPIO_config(MSS_GPIO_30 , MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW);
	MSS_GPIO_enable_irq(MSS_GPIO_30);
}

/*GPIO9服务函数*/
void GPIO9_IRQHandler(void)
{
	MSS_GPIO_clear_irq(MSS_GPIO_9);
	
	CANIP_ISR(0);
}

/*GPIO10服务函数*/
void GPIO10_IRQHandler(void)
{
	MSS_GPIO_clear_irq(MSS_GPIO_10);
	
	CANIP_ISR(1);
}

/*GPIO30服务函数*/
void GPIO30_IRQHandler(void)
{
	FPGA_REG_RW_2BYTE(OUT_PPS_IRQ_CLEAR_ADDR) = 0x5555;

	MSS_GPIO_clear_irq(MSS_GPIO_30);
}
