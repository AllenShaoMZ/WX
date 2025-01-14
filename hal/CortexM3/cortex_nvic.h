/*******************************************************************************
 * (c) Copyright 2007 Actel Corporation.  All rights reserved.
 * 
 * Legacy Actel HAL Cortex NVIC control functions.
 * The use of these functions should be replaced by calls to the equivalent
 * CMSIS function in your application code.
 *
 * SVN $Revision: 4825 $
 * SVN $Date: 2012-12-14 11:05:13 +0000 (Fri, 14 Dec 2012) $
 */
#ifndef CORTEX_NVIC_H_
#define CORTEX_NVIC_H_

#include <stdint.h>

typedef void (*interrupt_handler_t)(void);

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_init( void );

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_set_handler
(
    uint32_t interrupt_number,
    interrupt_handler_t handler
);

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_set_priority
(
    uint32_t interrupt_number,
    uint8_t priority_level
);

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_enable_interrupt( uint32_t interrupt_number );

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_disable_interrupt( uint32_t interrupt_number );

/*------------------------------------------------------------------------------
 * 
 */
void NVIC_clear_interrupt( uint32_t interrupt_number );

#endif /*CORTEX_NVIC_H_*/
