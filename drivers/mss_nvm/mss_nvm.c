/*******************************************************************************
 * (c) Copyright 2011 Microsemi SoC Products Group.  All rights reserved.
 *
 * This source file contains SmartFusion2 eNVM driver code.
 *
 * SVN $Revision: 4713 $
 * SVN $Date: 2012-10-04 11:45:53 +0100 (Thu, 04 Oct 2012) $
 */

#include "../../CMSIS/m2sxxx.h"
#include "../../CMSIS/mss_assert.h"
#include "mss_nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/* Preprocessor definitions                                               */
/**************************************************************************/
/*     eNVM command codes       */
#define PROG_ADS       0x08000000u  /* One shot program with data in WD */
#define VERIFY_ADS     0x10000000u  /* One shot verification with data in WD */
#define USER_UNLOCK    0x13000000u  /* User unlock */

#define BITS_PER_PAGE  1024u                   /* Number of bits per page */
#define BYTES_PER_PAGE (BITS_PER_PAGE / 8u)    /* Number of bytes per page */

#define NVM_OFFSET_SIGNIFICANT_BITS     0x0007FFFFu
#define NVM1_BOTTOM_OFFSET              0x00040000u
#define NVM1_TOP_OFFSET                 0x0007FFFFu

#define NVM_BASE_ADDRESS                0x60000000u

#define PAGE_ADDR_MASK                  0xFFFFFF80u

#define WD_WORD_SIZE        32u

#define NB_OF_BYTES_IN_A_WORD           4u

#define WRITE_ERROR_MASK    (MSS_NVM_VERIFY_FAIL | \
                             MSS_NVM_EVERIFY_FAIL | \
                             MSS_NVM_WVERIFY_FAIL | \
                             MSS_NVM_PEFAIL_LOCK | \
                             MSS_NVM_WRCNT_OVER | \
                             MSS_NVM_WR_DENIED)

/*******************************************************************************
 * Combined status definitions
 * Below definitions should be used to decoded the bit encoded status returned 
 * by the function MSS_NVM_get_status().
 */
#define MSS_NVM_BUSY_B              (1u)        /* NVM is performing an internal operation */
#define MSS_NVM_VERIFY_FAIL         ((uint32_t)1 << 1u)     /* NVM verify operation failed */
#define MSS_NVM_EVERIFY_FAIL        ((uint32_t)1 << 2u)     /* NVM erase verify operation failed */
#define MSS_NVM_WVERIFY_FAIL        ((uint32_t)1 << 3u)     /* NVM write verify operation failed */
#define MSS_NVM_PEFAIL_LOCK         ((uint32_t)1 << 4u)     /* NVM program / erase operation failed due to page lock */
#define MSS_NVM_WRCNT_OVER          ((uint32_t)1 << 5u)     /* NVM write count overflowed */
#define MSS_NVM_WR_DENIED           ((uint32_t)1 << 18u)    /* NVM write is denied due to protection */

/*******************************************************************************
 * 
 */
#define NVM_BLOCK_0     0u
#define NVM_BLOCK_1     1u

/* For a future revision of the driver. */
void NVM_unlock(void);

/**************************************************************************/
/* Global data definitions                                                */
/**************************************************************************/
/**************************************************************************//**
 * Look-up table for NVM blocks.
 */
static NVM_TypeDef * const g_nvm[] = 
{
    ENVM_1 ,
    ENVM_2
};

static NVM32_TypeDef * const g_nvm32[] =
{
    (NVM32_TypeDef *)ENVM1_BASE ,
    (NVM32_TypeDef *)ENVM2_BASE
};

/**************************************************************************/
/* Private function declarations                                          */
/**************************************************************************/
static nvm_status_t request_nvm_access
(
    uint32_t nvm_block_id
);

static nvm_status_t get_ctrl_access
(
    uint32_t nvm_offset,
    uint32_t  length
);

static void release_ctrl_access(void);

static uint32_t get_remaining_page_length
(
    uint32_t offset,
    uint32_t  length
);

static void fill_wd_buffer
(
    const uint8_t * p_data,
    uint32_t  length,
    uint32_t block,
    uint32_t offset
);

static uint32_t 
write_nvm
(
    uint32_t addr,
    const uint8_t * pidata,
    uint32_t  length,
    uint32_t  lock_page,
    uint32_t * p_status
);

static uint32_t 
wait_nvm_ready
(
    uint32_t block
);

static nvm_status_t get_error_code(uint32_t nvm_hw_status);

/**************************************************************************/
/* Public function definitions                                            */
/**************************************************************************/

/**************************************************************************//**
 * See mss_nvm.h for details of how to use this function.
 */
nvm_status_t 
NVM_write
(
    uint32_t addr,
    const uint8_t * pidata,
    uint32_t  length,
    uint32_t  lock_page
)
{
    nvm_status_t status;
    uint32_t nvm_offset;
    
    /*
     * Prevent pages being locked for now because current silicon does not
     * allow locked pages to be unlocked.
     */
    lock_page = NVM_DO_NOT_LOCK_PAGE;
    
    /* Ignore upper address bits to ignore remapping setting. */
    nvm_offset = addr & NVM_OFFSET_SIGNIFICANT_BITS;  /* Ignore remapping. */

    /* Gain exclusive access to eNVM controller */
    status = get_ctrl_access(nvm_offset, length);

    /* Write eNVM one page at a time. */
    if(NVM_SUCCESS == status)
    {
        uint32_t remaining_length = length;
        
        while((remaining_length > 0u) && (NVM_SUCCESS == status))
        {
            uint32_t length_written;
            uint32_t nvm_hw_status;
            
            length_written = write_nvm(addr + (length - remaining_length), 
                                       &pidata[length - remaining_length],
                                       remaining_length,
                                       lock_page,
                                       &nvm_hw_status);
            
            if(0u == length_written)
            {
                status = get_error_code(nvm_hw_status);
            }
            else if(remaining_length > length_written)
            {
                remaining_length -= length_written;
            }
            else
            {
                remaining_length = 0u;
            }
        }
        
        /* Release eNVM controllers so that other masters can gain access to it. */
        release_ctrl_access();
    }

    return status;
}

/**************************************************************************//**
  Generate error code based on NVM status value.
  
  The hardware nvm status passed as parameter is expected to be masked using the
  following mask:
                (MSS_NVM_VERIFY_FAIL | \
                 MSS_NVM_EVERIFY_FAIL | \
                 MSS_NVM_WVERIFY_FAIL | \
                 MSS_NVM_PEFAIL_LOCK | \
                 MSS_NVM_WRCNT_OVER | \
                 MSS_NVM_WR_DENIED)
  
 */
static nvm_status_t get_error_code(uint32_t nvm_hw_status)
{
    nvm_status_t status;
    if(nvm_hw_status & MSS_NVM_WR_DENIED)
    {
        status = NVM_PROTECTION_ERROR;
    }
    else if(nvm_hw_status & MSS_NVM_WRCNT_OVER)
    {
        status = NVM_WRITE_THRESHOLD_ERROR;
    }
    else if(nvm_hw_status & MSS_NVM_PEFAIL_LOCK)
    {
        status = NVM_PAGE_LOCK_ERROR;
    }
    else
    {
        status = NVM_VERIFY_FAILURE;
    }
    
    return status;
}

/**************************************************************************//**
 * See mss_nvm.h for details of how to use this function.
 */
void NVM_unlock(void)
{
    g_nvm[NVM_BLOCK_0]->CMD = USER_UNLOCK;
    g_nvm[NVM_BLOCK_1]->CMD = USER_UNLOCK;
}

/**************************************************************************/
/* Private function definitions                                            */
/**************************************************************************/

/**************************************************************************//**
 * 
 */
#define ACCESS_REQUEST_TIMEOUT      0x08000000u
#define REQUEST_NVM_ACCESS          0x01u
#define CORTEX_M3_ACCESS_GRANTED    0x05u

static uint8_t g_envm_ctrl_locks = 0x00u;

static nvm_status_t
request_nvm_access
(
    uint32_t nvm_block_id
)
{
    nvm_status_t status = NVM_SUCCESS;
    uint32_t timeout_counter = ACCESS_REQUEST_TIMEOUT;
    
    /*
     * Gain access to eNVM controller.
     */
    g_nvm[nvm_block_id]->REQ_ACCESS = REQUEST_NVM_ACCESS;
    while((g_nvm[nvm_block_id]->REQ_ACCESS != CORTEX_M3_ACCESS_GRANTED) 
          && (timeout_counter > 0u))
    {
        --timeout_counter;
        if(0u == timeout_counter)
        {
            status = NVM_IN_USE_BY_OTHER_MASTER;
        }
    }
    
    /*
     * Mark controller as locked is successful so that it will be unlocked by a
     * call to release_ctrl_access.
     */
    if(NVM_SUCCESS == status)
    {
        g_envm_ctrl_locks |= (uint8_t)((uint32_t)0x01 << nvm_block_id);
    }
    
    return status;
}


/**************************************************************************//**
 * 
 */
static nvm_status_t
get_ctrl_access
(
    uint32_t nvm_offset,
    uint32_t  length
)
{
    nvm_status_t access_req_status;
    
    /*
     * Gain access to eNVM controller(s).
     */
    if(nvm_offset < NVM1_BOTTOM_OFFSET)
    {
        access_req_status = request_nvm_access(NVM_BLOCK_0);
        if(NVM_SUCCESS == access_req_status)
        {
            uint32_t last_offset;
            last_offset = nvm_offset + length;
            if(last_offset >= NVM1_BOTTOM_OFFSET)
            {
                access_req_status = request_nvm_access(NVM_BLOCK_1);
                if(NVM_IN_USE_BY_OTHER_MASTER == access_req_status)
                {
                    release_ctrl_access();
                }
            }
        }
    }
    else
    {
        access_req_status = request_nvm_access(NVM_BLOCK_1);
    }
    
    return access_req_status;
}

/**************************************************************************//**
 * Release access to eNVM controllers.
 */
#define NVM_BLOCK_0_LOCK_MASK   0x01u
#define NVM_BLOCK_1_LOCK_MASK   0x02u

static void release_ctrl_access(void)
{
    uint8_t block_locked;
    
    block_locked = g_envm_ctrl_locks & NVM_BLOCK_0_LOCK_MASK;
    if(block_locked)
    {
        g_nvm[NVM_BLOCK_0]->REQ_ACCESS = 0x00u;
    }
    
    block_locked = g_envm_ctrl_locks & NVM_BLOCK_1_LOCK_MASK;
    if(block_locked)
    {
        g_nvm[NVM_BLOCK_1]->REQ_ACCESS = 0x00u;
    }
}

/**************************************************************************//**
 * 
 */
static uint32_t wait_nvm_ready(uint32_t block) 
{
    volatile uint32_t ctrl_status;
    uint32_t ctrl_ready;
    
    /*
     * Wait for NVM to become ready.
     * We must do one dummy read of the status register before we are sure the
     * returned status is valid.
     */
    ctrl_status = g_nvm[block]->STATUS;
    do {
        ctrl_status = g_nvm[block]->STATUS;
        ctrl_ready = ctrl_status  & MSS_NVM_BUSY_B;
    } while(!ctrl_ready);
    
    return ctrl_status;
}

/**************************************************************************//**
  Write as much data as will fit into the eNVM page corresponding to the
  address "addr" passed as parameter. Return the number of bytes written into
  the page.
  In case of error, return the content of the eNVM controller status register
  into the 32-bit word pointed to by p_status.
 */
static uint32_t 
write_nvm
(
    uint32_t addr,
    const uint8_t * pidata,
    uint32_t  length,
    uint32_t  lock_page,
    uint32_t * p_status
)
{
    uint32_t length_written;
    uint32_t offset;
    
    *p_status = 0u;
    
    offset = addr & NVM_OFFSET_SIGNIFICANT_BITS;  /* Ignore remapping. */

    ASSERT(offset <= NVM1_TOP_OFFSET);
    
    /* Adjust length to fit within one page. */
    length_written = get_remaining_page_length(offset, length);
    
    if(offset <= NVM1_TOP_OFFSET)
    {
        uint32_t block;
        volatile uint32_t ctrl_status;
        uint32_t errors;
        
        if(offset < NVM1_BOTTOM_OFFSET)
        {
            block = NVM_BLOCK_0;
        }
        else
        {
            block = NVM_BLOCK_1;
            offset = offset - NVM1_BOTTOM_OFFSET;
        }
        
        fill_wd_buffer(pidata, length_written, block, offset);

        /* Set requested locking option. */
        g_nvm[block]->PAGE_LOCK = lock_page;
        
        /* Issue program command */
        g_nvm[block]->CMD = PROG_ADS | (offset & PAGE_ADDR_MASK);
        
        /* Wait for NVM to become ready. */
        ctrl_status = wait_nvm_ready(block);

        /* Check for errors. */
        errors = ctrl_status & WRITE_ERROR_MASK;
        if(errors)
        {
            /* Signal that an error occured by returning 0 a a number of bytes written. */
            length_written = 0u;
            *p_status = g_nvm[block]->STATUS;
        }
        else
        {
            /* Perform a verify. */
            g_nvm[block]->CMD = VERIFY_ADS | (offset & PAGE_ADDR_MASK);
            /* Wait for NVM to become ready. */
            ctrl_status = wait_nvm_ready(block);

            /* Check for errors. */
            errors = ctrl_status & WRITE_ERROR_MASK;
            if(errors)
            {
                /* Signal that an error occured by returning 0 a a number of bytes written. */
                length_written = 0u;
                *p_status = g_nvm[block]->STATUS;
            }
        }
    }
    
    return length_written;
}

/*******************************************************************************
  Return the number of bytes between the offset location and the end of the page
  containing the first offset location. This tells us how many actual bytes can
  be programmed with a single ProgramADS command.
  This also tells us if we are programming a full page. If the return value is
  equal to BYTES_PER_PAGE then we will be pragramming an entire NVM page.
  Alternatively, this function returning a value other than BYTES_PER_PAGE
  indicates that the WD[] buffer will have to be seeded with the existing
  content of the eNVM before copying the data to program to eNVM into the WD[]
  buffer.
 */
static uint32_t get_remaining_page_length
(
    uint32_t offset,
    uint32_t  length
)
{
    uint32_t start_page_plus_one;
    uint32_t last_page;
    
    start_page_plus_one = (offset / BYTES_PER_PAGE) + 1u;
    last_page = (offset + length) / BYTES_PER_PAGE;
    if(last_page >= start_page_plus_one)
    {
        length = BYTES_PER_PAGE - (offset % BYTES_PER_PAGE);
    }
    
    return length;
}

/**************************************************************************//**
 * 
 */
static void fill_wd_buffer
(
    const uint8_t * p_data,
    uint32_t  length,
    uint32_t block,
    uint32_t offset
)
{
    uint32_t inc;
    uint32_t wd_offset;
    
    if(length != BYTES_PER_PAGE)
    {
        uint32_t * p_nvm32;
        uint32_t nb_non_written_words;
        uint32_t first_non_written_word;
        /* 
         * Fill beginning of WD[] with current content of NVM page that must not
         * be overwritten.
         */
        p_nvm32 = (uint32_t *)((NVM_BASE_ADDRESS + offset) & PAGE_ADDR_MASK);
        nb_non_written_words = (offset % BYTES_PER_PAGE) / NB_OF_BYTES_IN_A_WORD;
        if((offset % NB_OF_BYTES_IN_A_WORD) > 0u)
        {
            ++nb_non_written_words;
        }
        for(inc = 0u; (inc < WD_WORD_SIZE) && (inc < nb_non_written_words); ++inc)
        {
            g_nvm32[block]->WD[inc] = p_nvm32[inc];
        }
        
        /*
         * Fill end of WD[] with current content of NVM page that must not be
         * overwritten.
         */
        first_non_written_word = ((offset + length) % BYTES_PER_PAGE) / NB_OF_BYTES_IN_A_WORD;
        nb_non_written_words = (BYTES_PER_PAGE / NB_OF_BYTES_IN_A_WORD) - first_non_written_word;
        
        for(inc = 0u; inc < nb_non_written_words; ++inc)
        {
            g_nvm32[block]->WD[first_non_written_word + inc] = p_nvm32[first_non_written_word + inc];
        }
    }
    
    /*
     * Fill WD[] with data requested to be written into NVM.
     */
    wd_offset = offset % BYTES_PER_PAGE;
    for(inc = 0u; inc < length; ++inc)
    {
        g_nvm[block]->WD[wd_offset + inc] = p_data[inc];
    }
}

#ifdef __cplusplus
}
#endif

/******************************** END OF FILE ******************************/






