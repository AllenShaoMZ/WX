#ifndef _YC_INIT_H_
#define _YC_INIT_H_

//#include  <tm.h>

#define  TM_BAUD_SET_REG_ADDR    0x60000208								//??????????จน????
#define BAUD_8192   0x0C39     /*  25641027 /8192 -1  =3129//256:0C34	25600000*/
//#define BAUD_16384  0x0619      /*  25641027 /16384 -1  =1564//256:0619*/
void yc_init(void);


#endif
