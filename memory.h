#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <cpu.h>
#include <lib_mem.h>
#include "os.h" 

void SaveTo3(CPU_INT08U *pDst, CPU_INT08U *pSrc,CPU_INT16U len);
void SaveTo3_16(CPU_INT16U *pDst, CPU_INT16U wVal);
void SaveTo3_I16(CPU_INT16S *pDst, CPU_INT16S wVal);
void SaveTo3_32(CPU_INT32U *pDst, CPU_INT32U wVal);
void SaveTo3_I32(CPU_INT32S *pDst, CPU_INT32S wVal);
void SaveTo3_CPU_FP64(CPU_FP64 *pDst, CPU_FP64 wVal);

void getVarIn3_8(CPU_INT08U * pDst);
CPU_INT16U getVarIn3_16(CPU_INT16U * pDst);
CPU_INT16S getVarIn3_I16(CPU_INT16S * pDst);
CPU_INT32U getVarIn3_32(CPU_INT32U * pDst);
CPU_INT32S getVarIn3_I32(CPU_INT32S * pDst);

void  Memcpy8_16(void * _D, const void * _S, CPU_INT32U _N); 
void  Memcpy16_8(void * _D, const void * _S, CPU_INT32U _N);

void  NAND_reset(void);
void  NAND_init(void);
void  NAND_RDpageset(CPU_INT32U rdpage);
void  NAND_WTpageset(CPU_INT32U wtpage);
void  NAND_RDstart(void);
void  NAND_WTstart(void);
void  NAND_8Kwrite(CPU_INT32U wtpage,CPU_INT16U *NANDwtBuf);
void  NAND_8Kread(CPU_INT32U rdpage,CPU_INT16U *NANDrdBuf);

#define GLEN			0x00200000


#define NANDRSTADDR		0x64A04002
#define NANDRST			0x1234
#define NANDINTADDR		0x64A04004
#define NANDINT			0x5678
#define NANDCACHEADDR	0x64A00000
#define NANDCACHELEN	(0x3FFF/2)
#define NANDRDPAGEADDRH	0x64A04008	//高16位
#define NANDRDPAGEADDRL	0x64A0400A
#define NANDWTPAGEADDRH	0x64A0400C
#define NANDWTPAGEADDRL	0x64A0400E

#define NANDRDSTARTADDR	0x64A04020
#define NANDRDSTART		0x55AA
#define NANDWTSTARTADDR	0x64A04000
#define NANDWTSTART		0xAA55

#define NANDRDYCADDR	0x64A04002	//取每字节时，判bit1 为0才能取数
#define NANDBADADDR		0x64818000	//每个地址+2*N代表第N块的坏块状态 0好块


#endif

