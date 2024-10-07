/*
 * extern.h
 *
 *  Created on: 2023年10月13日
 *      Author: linxiaojun
 */

#ifndef SRC_HEAD_EXTERN_H_
#define SRC_HEAD_EXTERN_H_

//*********************************************************
//                      头文件
//*********************************************************
#include "typedef.h"
#include "FPGAUart.h"
#include "FPGACan.h" 

//*********************************************************
//                      全局变量声明
//*********************************************************
extern	T_ZJ_STATE g_ZJState;

//*********************************************************
//                      函数声明
//*********************************************************
void Time1_init(void);
void Timer1_IRQHandler(void);
void GPIO_init(void);
void GPIO9_IRQHandler(void);
void GPIO10_IRQHandler(void);
void System_HW_Init(void);
void System_SW_Init(void);

/******************************************************
 *                      basic.c                       *
 ******************************************************/
/*将两个Uint8拼成一个Uint16*/
Uint16 u8_2_u16(Uint8 ucData1,Uint8 ucData0);

/*将一个Uint16拆成Uint8*/
Uint8 u16_2_u8(Uint16 usData,Uint8 ucIndex);

/*4个8位组成32位*/
Uint32 u8_2_u32(Uint8 ucData3,Uint8 ucData2,Uint8 ucData1,Uint8 ucData0);

/*4个8位组成float*/
float u8_2_flt(Uint8 ucData3,Uint8 ucData2,Uint8 ucData1,Uint8 ucData0);

/*8个8位组成double*/
double u8_2_dbl(Uint8 aucData[8]);

/*将一个Uint32拆成Uint8*/
Uint8 u32_2_u8(Uint32 uiData,Uint8 ucIndex);

/*将一个float拆成Uint8*/
Uint8 flt_2_u8(float *fData,Uint8 ucIndex);

/*取值*/
Uint32 Get_Bit(Uint32 uData, Uint8 ucBitNum);

/*置位*/
void Set_Bit(Uint32 *uData, Uint8 ucBitNum);

/*清零*/
void Clear_Bit(Uint32 *uData, Uint8 ucBitNum);

/*交换两个数的值*/
void Swap_Value(Uint16 *pusdata1, Uint16 *pusdata2);

/*sign函数*/
Int32 sign(double dinput);

/*算正负数的次方*/
double pow_new(double dinput, double dpower);

/*8位异或校验和*/
Uint8 XorCheckSum8(Uint8 *pucStartAddr, Uint32 uiBytNum);

/*16位异或校验和*/
Uint16 XorCheckSum16(Uint16 *pusStartAddr, Uint32 uiBytNum);

/*32位异或校验和*/
Uint32 XorCheckSum32(Uint32 *puiData, Uint32 uiLength);

/*8位累加校验和*/
Uint8 AddCheckSum8(Uint8 *pucStartAddr, Uint32 uiBytNum);

/*16位累加校验和*/
Uint16 AddCheckSum16(Uint8 *pucStartAddr, Uint32 uiBytNum);

/*16位CRC校验*/
Uint16 CRC16(Uint8 *pucStartAddr, Uint32 uiBytNum);

Uint16 CRC16_2(Uint8 *pucStartAddr, Uint32 uiBytNum);

/*获取版本号，月，日*/
Uint16 GetVersion(void);

#endif /* SRC_HEAD_EXTERN_H_ */
