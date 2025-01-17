/*******************************************************************************
 * (c) Copyright 2007 Actel Corporation.  All rights reserved.
 * 
 * SVN $Revision: 4826 $
 * SVN $Date: 2012-12-14 11:33:20 +0000 (Fri, 14 Dec 2012) $
 */
#ifndef CORTEX_NVIC_REGS_H_
#define CORTEX_NVIC_REGS_H_

/*------------------------------------------------------------------------------
 * Interrupt Controller Type
 */
#define ICT_REG_OFFSET 0x00000004

/*------------------------------------------------------------------------------
 * SETENA
 */
#define SETENA_REG_OFFSET       0x00000100

/*------------------------------------------------------------------------------
 * CLRENA
 */
#define CLRENA_REG_OFFSET       0x00000180

/*------------------------------------------------------------------------------
 * SETPEND
 */
#define SETPEND_REG_OFFSET      0x00000200

/*------------------------------------------------------------------------------
 * CLRPEND
 */
#define CLRPEND_REG_OFFSET      0x00000280

/*------------------------------------------------------------------------------
 * PRIORITY_0
 */
#define PRIORITY_0_REG_OFFSET   0x00000400

/*------------------------------------------------------------------------------
 * PRIORITY_1
 */
#define PRIORITY_1_REG_OFFSET   0x00000404

/*------------------------------------------------------------------------------
 * PRIORITY_2
 */
#define PRIORITY_2_REG_OFFSET   0x00000408

/*------------------------------------------------------------------------------
 * PRIORITY_3
 */
#define PRIORITY_3_REG_OFFSET   0x0000040C

/*------------------------------------------------------------------------------
 * PRIORITY_4
 */
#define PRIORITY_4_REG_OFFSET   0x00000410

/*------------------------------------------------------------------------------
 * PRIORITY_5
 */
#define PRIORITY_5_REG_OFFSET   0x00000414

/*------------------------------------------------------------------------------
 * PRIORITY_6
 */
#define PRIORITY_6_REG_OFFSET   0x00000418

/*------------------------------------------------------------------------------
 * PRIORITY_7
 */
#define PRIORITY_7_REG_OFFSET   0x0000041C

#endif /*CORTEX_NVIC_REGS_H_*/
