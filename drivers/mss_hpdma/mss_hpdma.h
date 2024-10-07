/*******************************************************************************
 * (c) Copyright 2008-2010 Actel Corporation.  All rights reserved.
 * 
 *  SmartFusion2 MSS HPDMA bare metal software driver public API.
 *
 * SVN $Revision: 4544 $
 * SVN $Date: 2012-08-23 12:42:21 +0100 (Thu, 23 Aug 2012) $
 */

/*=========================================================================*//**
  @mainpage SmartFusion2 MSS HPDMA Bare Metal Driver.

  Following are key features supported by the SmartFusion2 MSS HPDMA driver:
    - Initialization
    - Control of descriptor transfer
    - Status of current transfers
    - Interrupt control functions

  @section intro_sec Introduction
  The SmartFusion2 Microcontroller Subsystem (MSS) includes a High performance direct
  memory access (HPDMA)controller which performs fast data transfer between
  SWITCH and DDR-Bridge memory.
  This software driver provides a set of functions for controlling the SmartFusion2
  MSS HPDMA as part of a bare metal system where no operating system is available.
  This driver can be adapted for use as part of an operating system but the
  implementation of the adaptation layer between this driver and the operating
  system's driver model is outside the scope of this driver.
  
  @section hw_dependencies Hardware Flow Dependencies
  The configuration of all features of the MSS HPDMA is covered by this driver.
  There are no dependencies on the hardware flow when configuring the SmartFusion2 MSS HPDMA.
  The base address and register addresses and interrupt number assignment for the
  MSS HPDMA block are defined as constants in the SmartFusion2 CMSIS-PAL. You must ensure
  that the SmartFusion2 CMSIS-PAL is either included in the software tool chain used to
  build your project or is included in your project.

  @section theory_op Theory of Operation

  The MSS HPDMA driver functions are grouped into the following categories:
    - Initialization
    - Control of descriptor transfer
    - Status of current transfer
    - Interrupt control functions

  Initialization
    The SmartFusion2 MSS HPDMA driver is initialized through a call to the MSS_HPDMA_init()
    function.The MSS_HPDMA_init() function must be called before any other HPDMA
    driver functions can be called.

  Control of descriptor transfer
    The different control functionalities of HPDMA are:
      - MSS_HPDMA_start() : It starts HPDMA transfer. You should provide source and
                            destination address along with transfer size and transfer
                            direction.
      - MSS_HPDMA_pause() : It pauses HPDMA transfer of the particular descriptor.
      - MSS_HPDMA_resume(): It resumes HPDMA transfer of the particular descriptor.
      - MSS_HPDMA_abort() : It aborts HPDMA transfer of the particular descriptor.

  Status of Current Transfer
    When there is any HPDMA transfer in progress, you can get the transfer state
    using function MSS_HPDMA_get_transfer_state(). Since HPDMA transfer involves bulk
    data transfer, an application can check how many bytes of data transfer
    are still pending using function MSS_HPDMA_get_pending_counters().

  Interrupt control functions
   The interrupts are controlled through following function:
     - MSS_HPDMA_enable_irq() : It enables either transfer interrupt or error
                                interrupt.
     - MSS_HPDMA_disable_irq(): It disables either transfer interrupt or error
                                interrupt.
     - MSS_HPDMA_clear_irq()  : It clears either transfer interrupt or error
                                interrupt.

  MSS_HPDMA_set_handler() function is provided to register a handler
  function which will be called by the driver when there is successful completion
  of DMA transfer or when there is an error in DMA transfer . You must create
  and register the handler function to suit your application.

  The function prototypes for the HPDMA interrupt handler is as follows:
    - void HPDMA_Complete_IRQHandler ( void )
    - void HPDMA_Error_IRQHandler ( void )

  Entry for these interrupt handler is provided in the SmartFusion2 CMSIS-PAL vector
  table. To add HPDMA error and complete interrupt handler, you must implement
  HPDMA_Error_IRQHandler() and HPDMA_Complete_IRQHandler()  functions as part of
  your application code.

 *//*=========================================================================*/
#ifndef MSS_HPDMA_H_
#define MSS_HPDMA_H_

#ifdef __cplusplus
extern "C" {
#endif 

#include "../../CMSIS/m2sxxx.h"

/*-------------------------------------------------------------------------*//**
  The HPDMA_SW_TO_DDR constant is used to specify the data transfer direction
  from Switch to DDR-bridge. This will be used as parameter by function
  MSS_HPDMA_configure().
 */
#define HPDMA_SW_TO_DDR                0u

/*-------------------------------------------------------------------------*//**
  The HPDMA_DDR_TO_SW constant is used to specify the data transfer direction
  from DDR-bridge to Switch. This will be used as parameter by function
  MSS_HPDMA_configure().
 */
#define HPDMA_DDR_TO_SW                1u

/*-------------------------------------------------------------------------*//**
  These constant are used for specifying the source and destination end for
  finding pending counters.
 */
#define HPDMA_SRC_END                  0u
#define HPDMA_DES_END                  1u

/*-------------------------------------------------------------------------*//**
  The HPMDA_MAX_DESCRIPTOR constant specifies maximum descriptor supported
  by HPDMA.
 */
#define HPMDA_MAX_DESCRIPTOR           4u

/*-------------------------------------------------------------------------*//**
  The hpdma_descriptor_t type is used to identify individual descriptor.
  They are used as argument to most SmartFusion2 MSS HPDMA driver functions to identify
  the descriptor on which action is taking place.
 */
typedef enum
{
    HPDMA_DESCRIPTOR_0,
    HPDMA_DESCRIPTOR_1,
    HPDMA_DESCRIPTOR_2,
    HPDMA_DESCRIPTOR_3,
    HPDMA_DESCRIPTOR_INVALID
}hpdma_descriptor_t;

/*-------------------------------------------------------------------------*//**
  The hpdma_xfr_state_t type identifies different HPDMA transfer states. Any
  descriptor is assumed to be in empty transfer state in idle condition. In
  Idle condition, descriptor is ready to be configured.
 */
typedef enum
{
    HPDMA_INVALID,
    HPDMA_EMPTY,
    HPDMA_XFR_ACTIVE,
    HPDMA_XFR_COMPLETE,
    HPDMA_XFR_SRC_ERROR,
    HPDMA_XFR_DES_ERROR
}hpdma_xfr_state_t;


/*-------------------------------------------------------------------------*//**
  The hpdma_irq_type_t specifies the different types of interrupt supported by
  HPDMA. However, there are only two interrupt channels. HPDMA_IRQ_XFR_ERR
  and HPDMA_IRQ_NON_WORD interrupts are routed through HPDMA_Error_IRQn.
 */
typedef enum
{
    HPDMA_IRQ_INVALID,
    HPDMA_IRQ_XFT_CMP ,
    HPDMA_IRQ_XFR_ERR,
    HPDMA_IRQ_NON_WORD
} hpdma_irq_type_t;

/*-------------------------------------------------------------------------*//**
  Interrupt handler prototype.
  This typedef specifies the prototype of functions that can be registered with
  this driver as interrupt handler functions.
 */
typedef void (*mss_hpdma_irq_handler_t)( hpdma_descriptor_t descriptor_id );

/*-------------------------------------------------------------------------*//**
  mss_hpdma_dcp_t
  There is one instance of this structure for each instance of the descriptor.
  Instance of this structure are used to identify a specific descriptor. This
  structure can be used to find out the source address, destination address
  and transfer size of the descriptor undergoing transfer.
*/

typedef struct mss_hpdma_dcp
{
    uint32_t src_addr; /*!< Source Address */
    uint32_t des_addr; /*!< Destination Address */
    uint32_t xfr_size; /*!< transfer size */
    uint8_t  xfr_dir; /*!< Transfer direction */

    mss_hpdma_irq_handler_t xfr_complete_irq;  /*!< Pointer to user registered transfer complete handler. */
    mss_hpdma_irq_handler_t xfr_error_irq;  /*!< Pointer to user registered transfer error handler. */
    mss_hpdma_irq_handler_t non_word_irq; /*!< Pointer to user registered non-word handler. */
}mss_hpdma_dcp_t;

extern mss_hpdma_dcp_t the_descriptor[HPMDA_MAX_DESCRIPTOR];
/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_init() function initializes the HPDMA driver by resetting
  and taking HPDMA controller out of reset. It also clears all pending
  interrupts. It initializes all four descriptors with default values. It
  disables interrupt related to the descriptors’ transfer.

  @param
    none

  @return
    none
 */
void
MSS_HPDMA_init
(
    void
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_start() function starts the data transfer of particular
  descriptor of HPDMA. On completing Descriptor transfer HPDMA controller,
  updates the status of the transfer and clears dcp_valid bit.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param src_addr
    The parameter src_addr is source end memory start address for a descriptor.
    This address must be word aligned at the start. And the same address is
    incremented on each successful transfer at the source end. HPDMA controller
    starts reading the data from source memory and transfers to destination
    memory.

  @param dest_addr
    The parameter dest_addr is destination end memory start address for a
    descriptor.
    This address must be word aligned at the start. And the same address is
    incremented on each successful transfer at the Destination end. HPDMA
    controller starts reading the data from source memory and transfers to
    destination memory.

  @param transfer_size
    The parameter transfer_size specify data transfer size of particular
    descriptor on the HPDMA channel.

  @param transfer_dir
     The parameter transfer_dir is used for specifying the data transfer
     direction.
     Following are two possible data transfer direction:
       HPDMA_SW_TO_DDR
       HPDMA_DDR_TO_SW

  @return
    none.
 */
void
MSS_HPDMA_start
(
    hpdma_descriptor_t descriptor_id,
    uint32_t src_addr,
    uint32_t dest_addr,
    uint32_t transfer_size,
    uint8_t transfer_dir
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_pause() function pause the descriptor transfer on HPDMA
  channel

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @return
    none.
 */
void
MSS_HPDMA_pause
(
    hpdma_descriptor_t descriptor_id
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_resume() function resumes the descriptor transfer on HPDMA
  channel

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @return
    none.
 */
void
MSS_HPDMA_resume
(
    hpdma_descriptor_t descriptor_id
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_abort() function aborts the descriptor transfer on HPDMA
  channel.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @return
    none.
 */
void
MSS_HPDMA_abort
(
    hpdma_descriptor_t descriptor_id
);
/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_get_transfer_state() function returns the transfer state
  of the descriptor. The possible states of data transfer are :
    HPDMA_INVALID
    HPDMA_EMPTY
    HPDMA_XFR_ACTIVE
    HPDMA_XFR_COMPLETE
    HPDMA_XFR_SRC_ERROR
    HPDMA_XFR_DES_ERROR

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @return
    It returns transfer state data of type hpdma_xfr_state_t. The possible
    state of data transfers are :
      HPDMA_INVALID
      HPDMA_EMPTY
      HPDMA_XFR_ACTIVE
      HPDMA_XFR_COMPLETE
      HPDMA_XFR_SRC_ERROR
      HPDMA_XFR_DES_ERROR
 */
hpdma_xfr_state_t
MSS_HPDMA_get_transfer_state
(
    hpdma_descriptor_t descriptor_id
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_get_pending_counters() function pending transfer counters
  in words for particular descriptor.
  The pending transfer counts can be for source and destination. if the value
  returned at the end of the transfer is zero, it indicates successful transfer
  and non zero value indicates error occurrence during the Descriptor transfer
  at source or destination end.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param src_des_end
    The parameter src_des_end specify the end on which we are interested
    in knowing the pending counters.
    The possible values are:
      HPDMA_SRC_END
      HPDMA_DES_END

  @return
    It returns pending transfer counters in words for particular descriptor at
    source or destination end.
 */
uint16_t
MSS_HPDMA_get_pending_counters
(
    hpdma_descriptor_t descriptor_id,
    uint8_t src_des_end
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_set_handler() function is used to register a handler
  function which will be called by the driver when there is successful completion
  of DMA transfer or when there is an error in DMA transfer . You must create
  and register the handler function to suit your application. The
  MSS_HPDMA_set_handler() function also enables either completed transfer or error
  interrupt in the Cortex-M3 NVIC as part of its implementation.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param handler
    The handler parameter is a pointer to a interrupt handler function
    provided by your application which is called as a result of any of the three
    interrupts. This handler is of mss_hpdma_irq_handler_t type.

  @param irq_type
    The parameter irq_type specify the type of IRQ which should be enabled.
    The type of IRQs are:
      HPDMA_IRQ_XFT_CMP
      HPDMA_IRQ_XFR_ERR
      HPDMA_IRQ_NON_WORD

  @return
    none
 */
void
MSS_HPDMA_set_handler
(
    hpdma_descriptor_t descriptor_id,
    mss_hpdma_irq_handler_t handler,
    hpdma_irq_type_t  irq_type
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_enable_irq() function is used to enable interrupt generation
  for the specified interrupt type for the specific descriptor.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param irq_type
    The parameter irq_type specify the type of IRQ which should be enabled.
    The type of IRQs are:
      HPDMA_IRQ_XFT_CMP
      HPDMA_IRQ_XFR_ERR
      HPDMA_IRQ_NON_WORD

  @return
    none
 */
void
MSS_HPDMA_enable_irq
(
    hpdma_descriptor_t descriptor_id,
    hpdma_irq_type_t irq_type
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_disable_irq() function is used to disable interrupt generation
  for the specified interrupt type of the specific descriptor .

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param irq_type
    The parameter irq_type specify the type of IRQ which should be enabled.
    The type of IRQs are:
      HPDMA_IRQ_XFT_CMP
      HPDMA_IRQ_XFR_ERR
      HPDMA_IRQ_NON_WORD

  @return
    none
 */
void
MSS_HPDMA_disable_irq
(
    hpdma_descriptor_t descriptor_id,
    hpdma_irq_type_t irq_type
);

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_clear_irq() function is used to clear interrupt for the
  specified interrupt type of the specific descriptor.

  @param descriptor_id
    The parameter descriptor_id specify the descriptor number. There are four
    descriptor supported by HPDMA.

  @param irq_type
    The parameter irq_type specify the type of IRQ which should be enabled.
    The type of IRQs are:
      HPDMA_IRQ_XFT_CMP
      HPDMA_IRQ_XFR_ERR
      HPDMA_IRQ_NON_WORD

  @return
    none
 */
void
MSS_HPDMA_clear_irq
(
    hpdma_descriptor_t descriptor_id,
    hpdma_irq_type_t irq_type
);

#ifdef __cplusplus
}
#endif

#endif /* MSS_HPDMA_H_ */
