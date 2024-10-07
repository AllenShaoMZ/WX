/*******************************************************************************
 * (c) Copyright 2012 Microsemi SoC Products Group.  All rights reserved.
 *
 * SmartFusion2 COMBLK access functions.
 *
 * SVN $Revision: 4676 $
 * SVN $Date: 2012-09-26 14:51:56 +0100 (Wed, 26 Sep 2012) $
 */
#ifndef __MSS_COMBLK_H_
#define __MSS_COMBLK_H_ 1

#include "m2sxxx.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 *
 */
typedef void(*comblk_completion_handler_t)(uint8_t * p_response, uint16_t response_size);

typedef uint32_t (*comblk_page_handler_t)(uint8_t const ** pp_next_page);


/*------------------------------------------------------------------------------
 *
 */
void MSS_COMBLK_init(void);

/*------------------------------------------------------------------------------
 *
 */
void MSS_COMBLK_send_cmd_with_ptr
(
    uint8_t cmd_opcode,
    uint32_t cmd_params_ptr,
    uint8_t * p_response,
    uint16_t response_size,
    comblk_completion_handler_t completion_handler
);

/*------------------------------------------------------------------------------
 *
 */
void MSS_COMBLK_send_cmd
(
    const uint8_t * p_cmd,
    uint16_t cmd_size,
    const uint8_t * p_data,
    uint32_t data_size,
    uint8_t * p_response,
    uint16_t response_size,
    comblk_completion_handler_t completion_handler
);

/*------------------------------------------------------------------------------
 *
 */
void MSS_COMBLK_read
(
    const uint8_t * p_data,
    uint16_t cmd_size,
    uint8_t * p_response,
    uint16_t response_size,
    comblk_completion_handler_t completion_handler
);

/*------------------------------------------------------------------------------
 *
 */
void MSS_COMBLK_send_paged_cmd
(
    const uint8_t * p_cmd,
    uint16_t cmd_size,
    uint8_t * p_response,
    uint16_t response_size,
    uint32_t (*)(uint8_t const **),
    void (*completion_handler)(uint8_t *, uint16_t)
);

#ifdef __cplusplus
}
#endif

#endif /* __MSS_COMBLK_H_ */
