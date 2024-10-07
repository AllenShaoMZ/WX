/*******************************************************************************
 * (c) Copyright 2012 Microsemi Corporation.  All rights reserved.
 * 
 *  SmartFusion2 CMSIS system initialization.
 *
 * SVN $Revision: 4337 $
 * SVN $Date: 2012-05-24 15:00:56 +0100 (Thu, 24 May 2012) $
 */

#ifndef __SYSTEM_M2SXXX_H__
#define __SYSTEM_M2SXXX_H__

#ifdef __cplusplus
extern "C" {
#endif 

/* Standard CMSIS global variables. */
extern uint32_t SystemFrequency;    /*!< System Clock Frequency (Core Clock) */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/* SmartFusion specific clocks. */
extern uint32_t g_FrequencyPCLK0;   /*!< Clock frequency of APB bus 0. */  
extern uint32_t g_FrequencyPCLK1;   /*!< Clock frequency of APB bus 1. */
extern uint32_t g_FrequencyACE;     /*!< Clock frequency of Analog Compute Engine. */
extern uint32_t g_FrequencyFPGA;    /*!< Clock frequecny of FPGA fabric */

/***************************************************************************//**
 * The SystemInit() is a standard CMSIS function called during system startup.
 * It is meant to perform low level hardware setup such as configuring PLLs. In
 * the case of SmartFusion these hardware setup operations are performed by the
 * chip boot which executed before the application started. Therefore this
 * function does not need to perform any hardware setup.
 */
void SystemInit(void);

/***************************************************************************//**
 * The SystemCoreClockUpdate() is a standard CMSIS function which can be called
 * by the application in order to ensure that the SystemCoreClock global
 * variable contains the up to date Cortex-M3 core frequency. Calling this
 * function also updates the global variables containing the frequencies of the
 * APB busses connecting the peripherals and the ACE frequency.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
