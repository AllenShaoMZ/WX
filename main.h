/*shao */

#ifndef	_MAIN_H_
#define	_MAIN_H_

/**************** 用户任务声明 *******************/

#include  <os.h>
#include  <stdint.h>

#define GC_ADDR 0x64000000
#define ZS_ADDR 0x64000300

static void AppTaskStart(void *p_arg);
static void MainTask(void *p_arg);
static void YkTask(void *p_arg);
static void YCTask(void *p_arg);
static void CanTask(void *p_arg);
static void AOCCTask(void *p_arg);
static void FLYTask(void *p_arg);

void extend100ms2(void);
extern void urgent_ck (void);

void (*func_sc_replay)(void); 	   		 
void (*func_inzs_handle)(CPU_INT16U length,CPU_INT08U * data); 		 
void (*func_inCmd_handle)(CPU_INT16U data);     

void (*func_ck_reset)(void);
void (*func_batt_over_protec )(void);

void (*func_beiyong_func)(void); 
void (*func_sc_cache)(void); 
void (*func_sc_real)(void);    


void SysTick_Init(uint32_t ticks);
void CanLX_send(void);
void XM_Task(void);

void ck_main_prog(void);

#endif
