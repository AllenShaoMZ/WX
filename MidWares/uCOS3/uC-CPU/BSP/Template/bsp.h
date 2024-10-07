/*
*********************************************************************************************************
*                                     MICRIUM BOARD SUPPORT SUPPORT
*
*                          (c) Copyright 2003-2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                      BOARD SUPPORT PACKAGE (BSP)
*
*                                      SMARTFUSION EVALUATION KIT
*
* Filename      : bsp.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_PRESENT
#define  BSP_PRESENT


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <cpu_core.h>

#include  <lib_def.h>
#include  <lib_ascii.h>

/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/


#ifdef   BSP_MODULE
#define  BSP_EXT
#else
#define  BSP_EXT  extern
#endif


/*
*********************************************************************************************************
*                                        DEFAULT CONFIGURATION
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                      CONDITIONAL INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_BOARD_NAME                         "SMARTFUSION2"

/*
*********************************************************************************************************
*                                         REGISTER DEFINES
*********************************************************************************************************
*/

#define  BSP_SYS_REG_BASE                                    ((CPU_INT32U)(0x40038000))

#define  BSP_SYS_REG_M3_CR                                   (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x4C)))
#define  BSP_SYS_REG_SOFT_RESET_CR                           (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x48)))
#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_LOW_CR                (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x90)))
#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_HIGH_CR               (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x94)))
#define  BSP_SYS_REG_MSSDDR_FACC1_CR                         (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x98)))
#define  BSP_SYS_REG_MSSDDR_FACC2_CR                         (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x9C)))
#define  BSP_SYS_REG_MSSDDR_PLL_STATUS                       (*((CPU_REG32 *)(BSP_SYS_REG_BASE + 0x150)))

#define  BSP_MSS_GPIO_BASE                                   ((CPU_INT32U)(0x40013000))

#define  BSP_MSS_GPIO_0_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x00)))
#define  BSP_MSS_GPIO_1_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x04)))
#define  BSP_MSS_GPIO_2_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x08)))
#define  BSP_MSS_GPIO_3_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x0C)))
#define  BSP_MSS_GPIO_4_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x10)))
#define  BSP_MSS_GPIO_5_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x14)))
#define  BSP_MSS_GPIO_6_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x18)))
#define  BSP_MSS_GPIO_7_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x1C)))
#define  BSP_MSS_GPIO_8_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x20)))
#define  BSP_MSS_GPIO_9_CFG                                  (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x24)))
#define  BSP_MSS_GPIO_10_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x28)))
#define  BSP_MSS_GPIO_11_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x2C)))
#define  BSP_MSS_GPIO_12_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x30)))
#define  BSP_MSS_GPIO_13_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x34)))
#define  BSP_MSS_GPIO_14_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x38)))
#define  BSP_MSS_GPIO_15_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x3C)))
#define  BSP_MSS_GPIO_16_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x40)))
#define  BSP_MSS_GPIO_17_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x44)))
#define  BSP_MSS_GPIO_18_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x48)))
#define  BSP_MSS_GPIO_19_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x4C)))
#define  BSP_MSS_GPIO_20_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x50)))
#define  BSP_MSS_GPIO_21_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x54)))
#define  BSP_MSS_GPIO_22_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x58)))
#define  BSP_MSS_GPIO_23_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x5C)))
#define  BSP_MSS_GPIO_24_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x60)))
#define  BSP_MSS_GPIO_25_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x64)))
#define  BSP_MSS_GPIO_26_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x68)))
#define  BSP_MSS_GPIO_27_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x6C)))
#define  BSP_MSS_GPIO_28_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x70)))
#define  BSP_MSS_GPIO_29_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x74)))
#define  BSP_MSS_GPIO_30_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x78)))
#define  BSP_MSS_GPIO_31_CFG                                 (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x7C)))

#define  BSP_MSS_GPIO_IN                                     (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x84)))
#define  BSP_MSS_GPIO_OUT                                    (*((CPU_REG32 *)(BSP_MSS_GPIO_BASE + 0x88)))

#define  BSP_MSS_TIMER1_BASE                                 ((CPU_INT32U)(0x40004000))

#define  BSP_MSS_TIMER1_VAL                                  (*((CPU_REG32 *)(BSP_MSS_TIMER1_BASE + 0x00)))
#define  BSP_MSS_TIMER1_LOADVAL                              (*((CPU_REG32 *)(BSP_MSS_TIMER1_BASE + 0x04)))
#define  BSP_MSS_TIMER1_CTRL                                 (*((CPU_REG32 *)(BSP_MSS_TIMER1_BASE + 0x0C)))

#define  BSP_MSS_SYS_CTRL_BASE                               ((CPU_INT32U)(0xE000E000))
#define  BSP_MSS_SYS_CTRL_CCR                                (*((CPU_REG32 *)(BSP_MSS_SYS_CTRL_BASE + 0xD14)))

/*
*********************************************************************************************************
*                                            BIT DEFINES
*********************************************************************************************************
*/

#define  BSP_SYS_REG_M3_CR_STCLK_DIVISOR                     0x0C000000

#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_GPOUT_31_24           0x01000000
#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_GPOUT_23_16           0x00800000
#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_GPOUT_15_8            0x00400000
#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_GPOUT_7_0             0x00200000
#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_GPIO                  0x00100000
#define  BSP_SYS_REG_SOFT_RESET_CR_MSS_FPGA                  0x00004000

#define  BSP_SYS_REG_MSSDDR_FACC1_CR_FACC_FAB_REF_SEL        0x08000000
#define  BSP_SYS_REG_MSSDDR_FACC1_CR_CONTROLLER_PLL_INIT     0x04000000
#define  BSP_SYS_REG_MSSDDR_FACC1_CR_FACC_GLMUX_SEL          0x00001000

#define  BSP_SYS_REG_MSSDDR_FACC2_CR_FACC_STANDBY_SEL        0x000001C0
#define  BSP_SYS_REG_MSSDDR_FACC2_CR_FACC_STANDBY_SEL_MUX2   0x00000100
#define  BSP_SYS_REG_MSSDDR_FACC2_CR_FACC_STANDBY_SEL_MUX1   0x00000080
#define  BSP_SYS_REG_MSSDDR_FACC2_CR_FACC_STANDBY_SEL_MUX0   0x00000040

#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_HIGH_CR_PLL_BYPASS    0x00000001

#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_ROSC_DIV2             0x00000004
#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_MPLL_LOCK             0x00000002
#define  BSP_SYS_REG_MSSDDR_PLL_STATUS_FAB_PLL_LOCK          0x00000001

#define  BSP_MSS_GPIO_CFG_OUT_REG_EN                         0x00000001
#define  BSP_MSS_GPIO_CFG_OUT_BUF_EN                         0x00000004

#define  BSP_MSS_TIMER1_CTRL_MODE                            0x00000002
#define  BSP_MSS_TIMER1_CTRL_EN                              0x00000001

#define  BSP_MSS_SYS_CTRL_CCR_STKALIGN                       0x00000200

/*
*********************************************************************************************************
*                                           CLOCK DEFINES
*********************************************************************************************************
*/

#define  BSP_MSS_SYS_CLK_FREQ_M3                             50000000
#define  BSP_MSS_SYS_CLK_FREQ_ROSC                           50000000
#define  BSP_MSS_SYS_CLK_FREQ_ROSC_1MHZ                       1000000
#define  BSP_MSS_SYS_CLK_FREQ_XTAL                              32768

/*
*********************************************************************************************************
*                                             TYPEDEFS
*********************************************************************************************************
*/

typedef  CPU_INT16U  BSP_DEV_NBR;
typedef  CPU_INT16U  BSP_OPT;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         BSP_PreInit            (void);
void         BSP_PostInit           (void);
void         BSP_LowLevelInit       (void);


/*
*********************************************************************************************************
*                                             CLOCK SERVICES
*********************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreqGet (void);
CPU_INT32U  BSP_APB_ClkFreqGet (void);

/*
*********************************************************************************************************
*                                             LED SERVICES
*********************************************************************************************************
*/

void         BSP_LED_Init           (void);
void         BSP_LED_On             (CPU_INT08U   led);
void         BSP_LED_Off            (CPU_INT08U   led);
void         BSP_LED_Toggle         (CPU_INT08U   led);


/*
*********************************************************************************************************
*                                          CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif    
