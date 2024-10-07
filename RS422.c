#include <os.h>

#include "RS422.h"

GNSS_TM	GNSS_buf_temp,GNSS_buf,GNSS_buf_old;
GNB_TM	GNB_buf_temp,GNB_buf;
XM_TM	XMA_buf_temp,XMA_buf;
XM_TM	XMB_buf_temp,XMB_buf;
FL_TM	FLX_buf_temp,FLX_buf;
FL_TM	FLY_buf_temp,FLY_buf;
FL_TM	FLZ_buf_temp,FLZ_buf;
DLX_TM	DLX_buf_temp,DLX_buf;
XKZ_TM	XKZ_buf_temp,XKZ_buf;
FL_TM	FLS_buf_temp,FLS_buf;

PCDU_TM	PCDU_buf_temp,PCDU_buf;
EP_TM	EP_buf_temp,EP_buf;

TL_TM	TLA_buf_temp,TLA_buf;
TL_TM	TLB_buf_temp,TLB_buf;
GNSS_R_TM	GNSS_R_buf_temp,GNSS_R_buf;
SBT_DCS_TM	SBT_DCS_buf_temp,SBT_DCS_buf;
XTTC_TM	XTTCA_buf_temp,XTTCA_buf;
XTTC_TM	XTTCB_buf_temp,XTTCB_buf;

//AOCC_DATA aocc_data;
//AOCC_TM *Aocc_TM_temp,Aocc_TM_buf;


CPU_FP32 discharge_cur=0.0;
