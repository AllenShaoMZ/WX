/*
 * define.h
 *
 *  Created on: 2023年10月13日
 *      Author: linxiaojun
 */

#ifndef SRC_HEAD_DEFINE_H_
#define SRC_HEAD_DEFINE_H_

//*********************************************************
//                      头文件
//*********************************************************
#include "include.h"

//*********************************************************
//                      宏定义
//*********************************************************
#define CSL_FMKR(msb, lsb, val) (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))
#define CSL_FINSR(reg, msb, lsb, val) ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb))) | CSL_FMKR(msb, lsb, val))
#define CSL_FEXTR(reg, msb, lsb) (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

#define	FPGA_REG_RW_1BYTE(addr)				*((volatile uint8_t *)(addr))		
#define	FPGA_REG_RW_2BYTE(addr)				*((volatile uint16_t *)(addr))
#define	FPGA_REG_RW_4BYTE(addr)				*((volatile uint32_t *)(addr))

#define ARMDOG_EN_ADDR          			*((volatile uint16_t *)0x301AFF30)

//串口寄存器定义
/*通信机*/
#define UART_TXJ_TXDATA_ADDR_DEBUG			(0x3012E000u)
#define UART_TXJ_CTRL_ADDR_DEBUG  			(0x3012E110u)
#define UART_TXJ_TXNUM_ADDR_DEBUG 			(0x3012E220u)
#define UART_TXJ_TXSTA_ADDR_DEBUG 			(0x3012E330u)
#define UART_TXJ_BOTELV_ADDR_DEBUG			(0x3012E440u)
#define UART_TXJ_RXDATA_ADDR_DEBUG			(0x3012F000u)
#define UART_TXJ_RXNUM_ADDR_DEBUG 			(0x3012F220u)
#define UART_TXJ_RXSTA_ADDR_DEBUG 			(0x3012F330u)
					
/*朝星*/					
#define UART_CX_TXDATA_ADDR_DEBUG 			(0x3014E000u)
#define UART_CX_CTRL_ADDR_DEBUG   			(0x3014E110u)
#define UART_CX_TXNUM_ADDR_DEBUG  			(0x3014E220u)
#define UART_CX_TXSTA_ADDR_DEBUG  			(0x3014E330u)
#define UART_CX_BOTELV_ADDR_DEBUG 			(0x3014E440u)
#define UART_CX_RXDATA_ADDR_DEBUG 			(0x3014F000u)
#define UART_CX_RXNUM_ADDR_DEBUG  			(0x3014F220u)
#define UART_CX_RXSTA_ADDR_DEBUG  			(0x3014F330u)
						
/*地测*/						
#define UART_DC_TXDATA_ADDR_DEBUG 			(0x3011E000u)
#define UART_DC_CTRL_ADDR_DEBUG   			(0x3011E110u)
#define UART_DC_TXNUM_ADDR_DEBUG  			(0x3011E220u)
#define UART_DC_TXSTA_ADDR_DEBUG  			(0x3011E330u)
#define UART_DC_BOTELV_ADDR_DEBUG 			(0x3011E440u)
#define UART_DC_RXDATA_ADDR_DEBUG 			(0x3011F000u)
#define UART_DC_RXNUM_ADDR_DEBUG  			(0x3011F220u)
#define UART_DC_RXSTA_ADDR_DEBUG  			(0x3011F330u)

/*惯组*/						
#define UART_GZ_TXDATA_ADDR_DEBUG 			(0x3015E000u)
#define UART_GZ_CTRL_ADDR_DEBUG   			(0x3015E110u)
#define UART_GZ_TXNUM_ADDR_DEBUG  			(0x3015E220u)
#define UART_GZ_TXSTA_ADDR_DEBUG  			(0x3015E330u)
#define UART_GZ_BOTELV_ADDR_DEBUG 			(0x3015E440u)
#define UART_GZ_RXDATA_ADDR_DEBUG 			(0x3015F000u)
#define UART_GZ_RXNUM_ADDR_DEBUG  			(0x3015F220u)
#define UART_GZ_RXSTA_ADDR_DEBUG  			(0x3015F330u)
		

//CAN总线锁存基址	
#define CAN_ALE_ADDR						(0x3019F9F0u)
			
//星箭分离接口基地址	
#define XJFL_RD1_ADDR						(0x301A5110u) //0x5555未分离，0xAAAA分离
#define XJFL_RD2_ADDR						(0x301A5220u) //0x5555未分离，0xAAAA分离

//自锁阀模块
#define	ZSF_ADDR							(0x301A5110u) //自锁阀1写0x0001,自锁阀2写0x0002

//控温模块
#define	KW1_ADDR							(0x301EF100u) //0xAAAA输出高电平，0x5555输出低电平
#define	KW2_ADDR							(0x301EF110u) 
#define	KW3_ADDR							(0x301EF120u) 
#define	KW4_ADDR							(0x301EF130u) 
#define	KW5_ADDR							(0x301EF140u) 
#define	KW6_ADDR							(0x301EF150u) 
#define	KW7_ADDR							(0x301EF160u) 
#define	KW8_ADDR							(0x301EF170u) 

#define	KW9_ADDR							(0x301EF200u) 
#define	KW10_ADDR							(0x301EF210u) 
#define	KW11_ADDR							(0x301EF220u) 
#define	KW12_ADDR							(0x301EF230u) 
#define	KW13_ADDR							(0x301EF240u) 
#define	KW14_ADDR							(0x301EF250u) 
#define	KW15_ADDR							(0x301EF260u) 
#define	KW16_ADDR							(0x301EF270u) 

#define	KW17_ADDR							(0x301EF300u) 
#define	KW18_ADDR							(0x301EF310u) 
#define	KW19_ADDR							(0x301EF320u) 
#define	KW20_ADDR							(0x301EF330u) 
#define	KW21_ADDR							(0x301EF340u) 

//电磁阀模块
//电磁阀1-8
#define	DCF1_ADDR							(0x301EFA00u) //单位0.1ms，时间可设置（0ms~1000ms），如果输入大于0x2710,则输出1s
#define	DCF2_ADDR							(0x301EFA10u)
#define	DCF3_ADDR							(0x301EFA20u)
#define	DCF4_ADDR							(0x301EFA30u)
#define	DCF5_ADDR							(0x301EFA40u)
#define	DCF6_ADDR							(0x301EFA50u)
#define	DCF7_ADDR							(0x301EFA60u)
#define	DCF8_ADDR							(0x301EFA70u)
//电磁阀9-16
#define	DCF9_ADDR							(0x301EFB00u)
#define	DCF10_ADDR							(0x301EFB10u)
#define	DCF11_ADDR							(0x301EFB20u)
#define	DCF12_ADDR							(0x301EFB30u)
#define	DCF13_ADDR							(0x301EFB40u)
#define	DCF14_ADDR							(0x301EFB50u)
#define	DCF15_ADDR							(0x301EFB60u)
#define	DCF16_ADDR							(0x301EFB70u)

//蓄电池控制
#define XDC_OPEN_ADDR						(0x301EF400u) //写0x3333
#define XDC_CLOSE_ADDR						(0x301EF410u) //写0x3333

//AD采集
/*GNSS（10K和4.7K分压）*/
#define	AD_GNSS_ADDR						(0x30199000u)
/*温度1-20*/
#define	AD_TEMP1_ADDR						(0x30199010u)
#define	AD_TEMP2_ADDR						(0x30199020u)
#define	AD_TEMP3_ADDR						(0x30199030u)
#define	AD_TEMP4_ADDR						(0x30199040u)
#define	AD_TEMP5_ADDR						(0x30199050u)
#define	AD_TEMP6_ADDR						(0x30199060u)
#define	AD_TEMP7_ADDR						(0x30199070u)
#define	AD_TEMP8_ADDR						(0x30199080u)
#define	AD_TEMP9_ADDR						(0x30199090u)
#define	AD_TEMP10_ADDR						(0x301990A0u)
#define	AD_TEMP11_ADDR						(0x301990B0u)
#define	AD_TEMP12_ADDR						(0x301990C0u)
#define	AD_TEMP13_ADDR						(0x301990D0u)
#define	AD_TEMP14_ADDR						(0x301990E0u)
#define	AD_TEMP15_ADDR						(0x301990F0u)
#define	AD_TEMP16_ADDR						(0x30199100u)
#define	AD_TEMP17_ADDR						(0x30199110u)
#define	AD_TEMP18_ADDR						(0x30199120u)
#define	AD_TEMP19_ADDR						(0x30199130u)
#define	AD_TEMP20_ADDR						(0x30199140u)
/*模拟量1-4*/
#define	AD_ANALOG1_ADDR						(0x30199150u)
#define	AD_ANALOG2_ADDR						(0x30199160u)
#define	AD_ANALOG3_ADDR						(0x30199170u)
#define	AD_ANALOG4_ADDR						(0x30199180u)
/*自锁阀1（10K和12K分压）*/
#define	AD_ZSF1_ADDR						(0x30199190u)
/*采集器1（10K和4.7K分压）*/
#define	AD_CJQ1_ADDR						(0x301991A0u)
/*载荷供电（10K和4.7K分压）*/
#define	AD_ZH_VOLTAGE_ADDR					(0x301991B0u)
/*7V电磁阀（20K和10K分压）*/
#define	AD_7V_DCF_ADDR						(0x301991C0u)
/*通信（10K和4.7K分压）*/
#define	AD_TX_ADDR							(0x301991D0u)
/*星敏（10K和4.7K分压）*/
#define	AD_XM_ADDR							(0x301991E0u)
/*惯组（10K和4.7K分压）*/
#define	AD_GZ_ADDR							(0x301991F0u)
/*温度21*/
#define	AD_TEMP21_ADDR						(0x30199200u)
/*3.3V*/
#define	AD_3_3V_ADDR						(0x30199210u)
/*充放电电流*/
#define	AD_XDC_CURRENT_ADDR					(0x30199220u)
/*蓄电池电压遥测（10K和2K分压）*/
#define	AD_XDC_VOLTAGE_ADDR					(0x30199230u)
/*放电开关状态*/
#define	AD_XDC_SWITCH_ADDR					(0x30199240u)

/*外部秒脉冲*/
#define	PPS_EN_DISABLE_ADDR					(0x301E8FF0u)
#define	OUT_PPS_IRQ_CLEAR_ADDR				(0x301AF120u)
#define	IN_PPS_IRQ_CLEAR_ADDR				(0x301AF110u)
                                                                                               
//蓄电池状态                                            
#define XDC_STATE_ADDR						(0x30199240u) //小于650  断

//主星422
#define	CX_422_OPEN_ADDR					(0x30144680u) //写0x0B55
#define	CX_422_CLOSE_ADDR					(0x30144680u) //写0x0BAA

#define	LEVEL_CMD_ADDR						(0x30144680u) //电平指令




#endif /* SRC_HEAD_DEFINE_H_ */
