;-------------------------------------------------------------------------------
; (c) Copyright 2007 Actel Corporation.  All rights reserved.
; 
; Interrupt disabling/restoration for critical section protection.
;
; SVN $Revision: 4826 $
; SVN $Date: 2012-12-14 11:33:20 +0000 (Fri, 14 Dec 2012) $
;
    AREA    |.text|, CODE, READONLY

;-------------------------------------------------------------------------------
; 
;
HAL_disable_interrupts \
    PROC
    mrs r0, PRIMASK
    cpsid I
    bx lr
    ENDP

;-------------------------------------------------------------------------------
;
;
HAL_restore_interrupts \
    PROC
    msr PRIMASK, r0
    bx lr
    ENDP

    END
