/*******************************************************************************
 * (c) Copyright 2008-2010 Actel Corporation.  All rights reserved.
 * 
 * SmartFusion2 MSS HPDMA bare metal driver implementation.
 *
 * SVN $Revision: 4544 $
 * SVN $Date: 2012-08-23 12:42:21 +0100 (Thu, 23 Aug 2012) $
 */
#include "mss_hpdma.h"
#include "../../CMSIS/mss_assert.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*-------------------------------------------------------------------------*//**
 * Different constants
 */
#define ENABLE    1u
#define DISABLE   0u
#define NULL_HANDLER  ( mss_hpdma_irq_handler_t )0

/* 64 K DMA Transfer */
#define MAX_SIZE_PER_DMA_XFR   0x10000u
#define MAX_IRQ_TYPE    3u

/*-------------------------------------------------------------------------*//**
 * Mask Constants
 */
#define HPDMA_BLOCK_ENABLE            0x2000u
#define HPDMA_SOFT_RESET              0x4000u
#define HPDMA_XFR_SIZE_MASK       0xFFFF0000u

/*-------------------------------------------------------------------------*//**
 * Data storage related to each descriptor
 */
mss_hpdma_dcp_t the_descriptor[HPMDA_MAX_DESCRIPTOR];

/*-------------------------------------------------------------------------*//**
 * Function prototype for local functions
 */
static hpdma_descriptor_t MSS_HPDMA_get_irq_source (hpdma_irq_type_t irq_type );
static void MSS_HPDMA_isr ( hpdma_descriptor_t dcp_id, hpdma_irq_type_t irq_type );

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_init
(
    void
)
{
    uint8_t dcp_number = 0u;
    uint8_t irq_type = 1u;
    // SYSREG->MSS_BLOCK_ENABLE_SR |= HPDMA_BLOCK_ENABLE;

    /* Reset HPDMA block */
    SYSREG->SOFT_RST_CR |= HPDMA_SOFT_RESET;

    /* Clear any previously pended MSS HPDMA interrupt */
    NVIC_ClearPendingIRQ( HPDMA_Error_IRQn );
    NVIC_ClearPendingIRQ(HPDMA_Complete_IRQn);

    /* Take HPDMA controller out of reset*/
    SYSREG->SOFT_RST_CR &= ~HPDMA_SOFT_RESET;

    /* Initialize data structures for each descriptors */
    while (dcp_number < HPMDA_MAX_DESCRIPTOR)
    {
        the_descriptor[dcp_number].xfr_size = 0 ;
        the_descriptor[dcp_number].xfr_complete_irq = NULL_HANDLER ;
        the_descriptor[dcp_number].xfr_error_irq = NULL_HANDLER ;
        the_descriptor[dcp_number].non_word_irq = NULL_HANDLER ;

        /* Disable and clear all type of interrupts */
        while ( irq_type <= MAX_IRQ_TYPE)
        {
            MSS_HPDMA_disable_irq( (hpdma_descriptor_t)dcp_number, (hpdma_irq_type_t)irq_type);
            MSS_HPDMA_clear_irq( (hpdma_descriptor_t)dcp_number, (hpdma_irq_type_t)irq_type);
            irq_type ++ ;
        }
        dcp_number++ ;
    }
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_start
(
    hpdma_descriptor_t descriptor_id,
    uint32_t src_addr,
    uint32_t dest_addr,
    uint32_t transfer_size,
    uint8_t transfer_dir
)
{
    /* Pause transfer. */
    MSS_HPDMA_pause( descriptor_id );

    /* Set descriptor transfer direction */
    HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_DCP_XFR_DIR = transfer_dir ;

    /* Store Source and destination Address */
    HPDMA->Descriptor[descriptor_id].HPDMASAR_REG = src_addr ;
    HPDMA->Descriptor[descriptor_id].HPDMADAR_REG = dest_addr ;

    /* Set the transfer size to 64K */
    HPDMA->Descriptor[descriptor_id].HPDMACR_REG &= HPDMA_XFR_SIZE_MASK ;

    if ( transfer_size <= MAX_SIZE_PER_DMA_XFR)
    {
        /* Set descriptor transfer size */
        HPDMA->Descriptor[descriptor_id].HPDMACR_REG |= (uint16_t)transfer_size ;
        the_descriptor[descriptor_id].xfr_size = 0;
    }
    else
    {
        the_descriptor[descriptor_id].xfr_size = transfer_size - MAX_SIZE_PER_DMA_XFR;
        the_descriptor[descriptor_id].des_addr = src_addr + MAX_SIZE_PER_DMA_XFR * sizeof(uint16_t);
        the_descriptor[descriptor_id].src_addr = dest_addr + MAX_SIZE_PER_DMA_XFR * sizeof(uint16_t);
        the_descriptor[descriptor_id].xfr_dir = transfer_dir;
    }

    /* Set valid descriptor to start transfer */
    HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_DCP_VALID = ENABLE ;

    /* Start transfer */
    MSS_HPDMA_resume( descriptor_id );
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_pause
(
    hpdma_descriptor_t descriptor_id
)
{
    /* Pause transfer */
    HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_DCP_PAUSE = ENABLE ;
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_resume
(
    hpdma_descriptor_t descriptor_id
)
{
    /* Resume transfer */
    HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_DCP_PAUSE = DISABLE ;
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_abort
(
    hpdma_descriptor_t descriptor_id
)
{
    HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_DCP_CLR = ENABLE ;
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
hpdma_xfr_state_t
MSS_HPDMA_get_transfer_state
(
    hpdma_descriptor_t descriptor_id
)
{
    hpdma_xfr_state_t current_state = HPDMA_INVALID ;

    if( HPDMA_BITBAND->Descriptor[descriptor_id].HPDMASR_DCP_ACTIVE)
    {
        current_state = HPDMA_XFR_ACTIVE ;
    }
    else if ( HPDMA_BITBAND->Descriptor[descriptor_id].HPDMASR_DCP_CMPLET )
    {
        current_state = HPDMA_XFR_COMPLETE ;
    }
    else if (  HPDMA_BITBAND->Descriptor[descriptor_id].HPDMASR_DCP_SERR )
    {
        current_state = HPDMA_XFR_SRC_ERROR ;
    }
    else if (  HPDMA_BITBAND->Descriptor[descriptor_id].HPDMASR_DCP_DERR )
    {
        current_state = HPDMA_XFR_DES_ERROR ;
    }

    if (current_state == HPDMA_INVALID )
    {
        if ( HPDMA_BITBAND->HPDMAEDR_DCP_EMPTY[descriptor_id])
        {
            current_state = HPDMA_EMPTY ;
        }
    }
    return( current_state );
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
uint16_t
MSS_HPDMA_get_pending_counters
(
    hpdma_descriptor_t descriptor_id,
    uint8_t src_des_end
)
{
    uint16_t pending_cntrs = 0;
    if ( (src_des_end == HPDMA_SRC_END) || (src_des_end == HPDMA_DES_END) )
    {
        pending_cntrs = (uint16_t)((HPDMA->Descriptor[descriptor_id].HPDMAPTR_REG)>>(16*src_des_end))  ;
    }

    return( pending_cntrs );
}

/*-------------------------------------------------------------------------*//**
 * See "mss_hpdma.h" for details of how to use this function.
 */
void MSS_HPDMA_set_handler
(
    hpdma_descriptor_t descriptor_id,
    mss_hpdma_irq_handler_t handler,
    hpdma_irq_type_t  irq_type
)
{
    if( handler == NULL_HANDLER )
    {
        /* Disable Interrupt related to transfer completed */
        MSS_HPDMA_disable_irq(descriptor_id , irq_type );

    }
    else
    {
        /* Enable Interrupt related to transfer completed */
        MSS_HPDMA_enable_irq(descriptor_id , irq_type );

        switch(irq_type)
        {
            case HPDMA_IRQ_XFT_CMP:
            {
                the_descriptor[descriptor_id].xfr_complete_irq =  handler ;
                break;
            }
            case HPDMA_IRQ_XFR_ERR:
            {
                the_descriptor[descriptor_id].xfr_error_irq =  handler ;
                break ;
            }
            case HPDMA_IRQ_NON_WORD:
            {
                the_descriptor[descriptor_id].non_word_irq =  handler ;
                break ;
            }
            default :
            {
                the_descriptor[descriptor_id].xfr_complete_irq =  NULL_HANDLER ;
                the_descriptor[descriptor_id].xfr_error_irq =  NULL_HANDLER ;
                the_descriptor[descriptor_id].non_word_irq =  NULL_HANDLER ;
                break ;
            }
        }
    }
}

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_get_irq_source() function returns the irq source of the
  different type of interrupt. If interrupt is HPDMA_Complete_IRQn,
  then this API is used to find out which descriptor has triggered the
  interrupt.
 */
hpdma_descriptor_t
MSS_HPDMA_get_irq_source
(
    hpdma_irq_type_t  irq_type
)
{
    hpdma_descriptor_t descriptor_id = HPDMA_DESCRIPTOR_INVALID;
    uint8_t dcp_id =0;

    switch(irq_type)
    {
        case HPDMA_IRQ_XFT_CMP:
        {
            while(dcp_id < HPMDA_MAX_DESCRIPTOR)
            {
                if( HPDMA_BITBAND->Descriptor[dcp_id].HPDMASR_DCP_CMPLET)
                {
                    descriptor_id = (hpdma_descriptor_t)dcp_id ;
                    break ;
                }
                ++ dcp_id ;
            }
            break ;
        }

        case HPDMA_IRQ_XFR_ERR:
        {
            while(dcp_id < HPMDA_MAX_DESCRIPTOR)
            {
                if( ( HPDMA_BITBAND->Descriptor[dcp_id].HPDMASR_DCP_DERR)
                                         ||
                     ( HPDMA_BITBAND->Descriptor[dcp_id].HPDMASR_DCP_SERR))
                {
                    descriptor_id = (hpdma_descriptor_t)dcp_id ;
                    break ;
                }
                ++ dcp_id ;
            }
            break ;
        }

        case HPDMA_IRQ_NON_WORD:
        {
            while(dcp_id < HPMDA_MAX_DESCRIPTOR)
            {
                if( HPDMA_BITBAND->HPDMAEDR_DCP_NON_WORD_ERR[dcp_id])
                {
                    descriptor_id = (hpdma_descriptor_t)dcp_id ;
                    break ;
                }
                ++ dcp_id ;
            }
            break ;
        }

        default:
            descriptor_id = HPDMA_DESCRIPTOR_INVALID ;
            break;
    }
    return descriptor_id;
}

/*-------------------------------------------------------------------------*//**
  See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_enable_irq
(
    hpdma_descriptor_t descriptor_id ,
    hpdma_irq_type_t  irq_type
)
{
    /* Enable interrupt */
    switch(irq_type)
    {
        case HPDMA_IRQ_XFT_CMP:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_CMP_INT = ENABLE;
            NVIC_EnableIRQ( HPDMA_Complete_IRQn );
            break ;
        }
        case HPDMA_IRQ_XFR_ERR:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_ERR_INT = ENABLE;
            NVIC_EnableIRQ( HPDMA_Error_IRQn );
            break;
        }
        case HPDMA_IRQ_NON_WORD:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_NON_WORD_INT = ENABLE;
            NVIC_EnableIRQ( HPDMA_Error_IRQn );
            break;
        }
        default:
            break;
    }
}

/*-------------------------------------------------------------------------*//**
  See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_disable_irq
(
    hpdma_descriptor_t descriptor_id ,
    hpdma_irq_type_t  irq_type
)
{
    uint8_t temp_dcp_id = 0;
    uint8_t irq_active = 0;

    /* Disable interrupt */
    switch(irq_type)
    {
        case HPDMA_IRQ_XFT_CMP:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_CMP_INT = DISABLE;

            while(temp_dcp_id < HPMDA_MAX_DESCRIPTOR)
            {
                if (HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_CMP_INT)
                {
                    irq_active = 1;
                    break ;
                }
                temp_dcp_id++ ;
            }
            break ;
        }
        case HPDMA_IRQ_XFR_ERR:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_ERR_INT = DISABLE;
            break;
        }
        case HPDMA_IRQ_NON_WORD:
        {
            HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_NON_WORD_INT = DISABLE;
            break;
        }
        default:
            break;
    }

    if ( (irq_type == HPDMA_IRQ_XFR_ERR) || (irq_type == HPDMA_IRQ_NON_WORD)  )
    {
        while(temp_dcp_id < HPMDA_MAX_DESCRIPTOR)
        {
            if(
                (HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_XFR_CMP_INT)
                                            ||
                (HPDMA_BITBAND->Descriptor[descriptor_id].HPDMACR_NON_WORD_INT)
               )
            {
                irq_active = 1;
                break ;
            }
            temp_dcp_id++ ;
        }
    }
}

/*-------------------------------------------------------------------------*//**
  See "mss_hpdma.h" for details of how to use this function.
 */
void
MSS_HPDMA_clear_irq
(
    hpdma_descriptor_t descriptor_id ,
    hpdma_irq_type_t  irq_type
)
{
    /* clear interrupt */
    switch(irq_type)
    {
        case HPDMA_IRQ_XFT_CMP:
        {
            HPDMA_BITBAND->HPDMAICR_CLR_XFR_INT[descriptor_id] = ENABLE;
            break ;
        }
        case HPDMA_IRQ_XFR_ERR:
        case HPDMA_IRQ_NON_WORD :
        {
            /* Check it again */
            HPDMA_BITBAND->HPDMAICR_NON_WORD_INT[descriptor_id] = ENABLE;
            break;
        }
        default:
            break;
    }
 //   NVIC_ClearPendingIRQ( HPDMA_Error_IRQn );
}

/*-------------------------------------------------------------------------*//**
  The MSS_HPDMA_isr() function is called by HPDMA_Complete_IRQhandler
  and HPDMA_error_IRQhandler
 */
void
MSS_HPDMA_isr
(
    hpdma_descriptor_t dcp_id,
    hpdma_irq_type_t irq_type
)
{
    switch( irq_type )
    {
        case HPDMA_IRQ_XFT_CMP:
        {
            if ( the_descriptor[dcp_id].xfr_size > 0 )
            {
                MSS_HPDMA_start
                (
                    dcp_id,
                    the_descriptor[dcp_id].src_addr,
                    the_descriptor[dcp_id].des_addr,
                    the_descriptor[dcp_id].xfr_size,
                    the_descriptor[dcp_id].xfr_dir
                );
            }
            else
            {
                /* Disable Interrupt related to transfer completed */
                MSS_HPDMA_disable_irq(dcp_id , irq_type );
            }
            if( the_descriptor[dcp_id].xfr_complete_irq != NULL_HANDLER)
            {
                (*(the_descriptor[dcp_id].xfr_complete_irq))( dcp_id );
            }
            break ;
        }
        case HPDMA_IRQ_XFR_ERR:
        {
            if( the_descriptor[dcp_id].xfr_error_irq != NULL_HANDLER)
            {
                (*(the_descriptor[dcp_id].xfr_error_irq))( dcp_id );
            }
            break ;
        }
        case HPDMA_IRQ_NON_WORD:
        {
            if( the_descriptor[dcp_id].non_word_irq != NULL_HANDLER)
            {
                (*(the_descriptor[dcp_id].non_word_irq))( dcp_id );
            }
            break ;
        }
        default :
        {
            break ;
        }
    }
}

/*------------------------------------------------------------------------------
  HPDMA Transfer complete service routine.
  This function will be called as a result of any descriptor transfer
  is completed by HPDMA controller .
 */
#if defined(__GNUC__)
__attribute__((__interrupt__)) void HPDMA_Complete_IRQHandler( void )
#else
void HPDMA_Complete_IRQHandler( void )
#endif 
{
    hpdma_descriptor_t desc_source = HPDMA_DESCRIPTOR_INVALID ;

    desc_source = MSS_HPDMA_get_irq_source( HPDMA_IRQ_XFT_CMP );

    if( desc_source != HPDMA_DESCRIPTOR_INVALID )
    {
        /* Clear interrupt */
        MSS_HPDMA_clear_irq(desc_source , HPDMA_IRQ_XFT_CMP );

        MSS_HPDMA_isr( desc_source, HPDMA_IRQ_XFT_CMP );
    }

    NVIC_ClearPendingIRQ( HPDMA_Complete_IRQn  );
}

/*------------------------------------------------------------------------------
  HPDMA Transfer Error service routine.
  This function will be called as a result of any descriptor transfer
  has an error condition or it there is non word transfer size error.
 */
#if defined(__GNUC__)
__attribute__((__interrupt__)) void HPDMA_Error_IRQHandler( void )
#else
void HPDMA_Error_IRQHandler( void )
#endif 
{
    hpdma_descriptor_t desc_source = HPDMA_DESCRIPTOR_INVALID ;

    desc_source = MSS_HPDMA_get_irq_source( HPDMA_IRQ_XFR_ERR );

    if( desc_source != HPDMA_DESCRIPTOR_INVALID )
    {
        /* Clear interrupt */
        MSS_HPDMA_clear_irq(desc_source , HPDMA_IRQ_XFR_ERR );

        MSS_HPDMA_isr( desc_source, HPDMA_IRQ_XFR_ERR );

    }
    else
    {
        desc_source = MSS_HPDMA_get_irq_source( HPDMA_IRQ_NON_WORD );

        if( desc_source != HPDMA_DESCRIPTOR_INVALID )
        {
            /* Clear interrupt */
            MSS_HPDMA_clear_irq(desc_source , HPDMA_IRQ_NON_WORD );

            MSS_HPDMA_isr( desc_source, HPDMA_IRQ_NON_WORD );
        }
    }

    NVIC_ClearPendingIRQ( HPDMA_Error_IRQn  );
}

#ifdef __cplusplus
}
#endif

