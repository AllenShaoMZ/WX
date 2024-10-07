/*******************************************************************************
 * (c) Copyright 2011 Microsemi SoC Products Group.  All rights reserved.
 * 
 * This file contains public APIs for SmartFusion2 eNVM software driver.
 * 
 * SVN $Revision: 4713 $
 * SVN $Date: 2012-10-04 11:45:53 +0100 (Thu, 04 Oct 2012) $
 */
/*=========================================================================*//**
  @mainpage SmartFusion2 MSS eNVM Bare Metal Driver.
  
  @section intro_sec Introduction
  The SmartFusion2 MicroController Subsystem (MSS) includes up to two embedded
  non-volatile memory (eNVM) blocks. Each of these eNVM blocks can be of a
  maximum size of 256KB. 

  This driver provides a set of functions for accessing and controlling the 
  MSS eNVMs as part of a bare metal system where no operating system is 
  available. These drivers can be adapted for use as part of an operating 
  system but the implementation of the adaptation layer between this driver 
  and the operating system's driver model is outside the scope of this driver.
  
  @section features Features
  The SmartFusion2 MSS eNVM driver provides the following features:
    •    Write operations.
  The SmartFusion2 MSS eNVM driver is provided as C source code.
  
  @section configuration Driver Configuration
  The configuration of all features of the MSS eNVMs is covered by this driver
  with the exception of master enable, deep power down mode and protection 
  configuration. These features are configured at the time of factory 
  programming. The base address, register addresses and interrupt number 
  assignment for the MSS eNVM blocks are defined as constants in the SmartFusion2 
  CMSIS-PAL You must ensure that the latest SmartFusion2 CMSIS-PAL is either included 
  in the software tool chain used to build your project or is included 
  in your project.

  @section theory_op Theory of Operation
  The MSS eNVM driver uses the SmartFusion2 "Cortex Microcontroler Software
  Interface Standard - Peripheral Access Layer" (CMSIS-PAL) to access hardware
  registers. You must ensure that the SmartFusion2 CMSIS-PAL is either included
  in the software toolchain used to build your project or is included in your
  project. The most up to date SmartFusion2 CMSIS-PAL files can be obtained using
  the Actel Firmware Catalog.

  The SmartFusion2 eNVM driver treats the two eNVM blocks as contiguous memory space 
  of size 512KB. As the eNVM hardware is divided in to 64 sectors per block, 
  each sector containing 32 pages and each page containing 1024bits, the 
  driver supports total 4096 pages in a contiguous  fashion. But the driver 
  imposes no restrictions while writing / reading / verifying data across 
  the pages. The driver supports random access method of accessing eNVM memory.
  
  The MSS eNVM driver API functions are categorized as below:
    •    Write / Program.

  All of these functions are blocking functions hence they return only if the 
  execution is successful or failed.

  The write function NVM_write() enables the user to write or program the 
  eNVM over any memory range within the limits of 512KB. The starting and 
  ending address of the memory range need not to be page aligned. Though the 
  eNVM hardware supports writing or programming operation without input data, 
  the write function does not support this feature hence mandates to provide 
  the input data to program. 

 *//*=========================================================================*/
#ifndef __MSS_NVM_H
#define __MSS_NVM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/* Public definitions                                                     */
/**************************************************************************/
/*******************************************************************************
 * 
 */
#define NVM_DO_NOT_LOCK_PAGE    0u
#define NVM_LOCK_PAGE           1u

/*******************************************************************************
  The nvm_status_t typedef is specifies the possible return values from
  function NVM_write().
    NVM_SUCCESS:
      Indicates that the programming was successful.
        
    NVM_PROTECTION_ERROR:
      Indicates that the operation could not be completed because of a protection
      error. This happens when attempting to program a page that was set as
      protected in the ahrdware flow.
      
    NVM_VERIFY_FAILURE:
      Indicates that one of the verify operations failed.
      
    NVM_PAGE_LOCK_ERROR:
      Indicates that the operation could not complete because one of the pages
      is locked. This may happen if the page was locked during a previous call to
      NVM_write() or if the page was locked in the hardware design flow.
      
    NVM_WRITE_THRESHOLD_ERROR:
      Indicates that the NVM maximum number of programming cycles has been reached.
      
    NVM_IN_USE_BY_OTHER_MASTER:
      Indicates that some other Comms Matrix master is accessing the NVM. This
      could be caused by FPGA logic or the system controller programming the NVM.
 */
typedef enum nvm_status
{
    NVM_SUCCESS = 0,
    NVM_PROTECTION_ERROR,
    NVM_VERIFY_FAILURE,
    NVM_PAGE_LOCK_ERROR,
    NVM_WRITE_THRESHOLD_ERROR,
    NVM_IN_USE_BY_OTHER_MASTER
} nvm_status_t;

/**************************************************************************/
/* Public variables                                                       */
/**************************************************************************/


/**************************************************************************/
/* Public function declarations                                           */
/**************************************************************************/

/***************************************************************************//**
  The function NVM_write() is used to program data in to the eNVM.
  This function treats the two eNVM blocks contiguously, hence 512KB of memory
  can be accessed linearly. The starting address and ending address of the memory
  to be written are need not be page aligned. This function supports
  programming data that spawns across multiple pages. This function does not
  support writing or programming eNVM without input data. This function is a
  blocking function.
  The NVM_write() function performs a verify operation on each page programmed
  to ensure the NVM is programmed with the expected content.

  @param addr
    The addr parameter is byte aligned starting address of the eNVM from where
    the data is to be programmed.
    
  @param pidata
    The pidata parameter is byte aligned starting address of the input data.
    
  @param len
    The len parameter is the number of bytes of data to be programmed.

  @param lock_page
    The lock_page parameter specifies if the page that are programmed should be
    lockec once programmed. Locking the pages will prevent the programmed pages
    to be overwritten by mistake. Subsequent programming of these pages will
    require the pages to be unlocked prior to calling NVM_write().
  
  @return
    This function returns NVM_SUCCESS on successful execution. It returns one
    of the following error codes if the programming of the NVM fails:
     - NVM_PROTECTION_ERROR
     - NVM_VERIFY_FAILURE
     - NVM_PAGE_LOCK_ERROR
     - NVM_WRITE_THRESHOLD_ERROR
     - NVM_IN_USE_BY_OTHER_MASTER

 */
nvm_status_t 
NVM_write
(
    uint32_t addr,
    const uint8_t * pidata,
    uint32_t  len,
    uint32_t  lock_page
);

#ifdef __cplusplus
}
#endif

#endif /* __MSS_NVM_H */


