/*
 * can.h
 *
 *  Created on: 2019年11月29日
 *      Author: Administrator
 */

#ifndef CAN_H_
#define CAN_H_

//#include "app_struct.h"
//#include "app_def.h"
#include "mss_can.h"

//*********************************************************
//                      宏定义
//*********************************************************

//*********************************************************
//                      类型重定义
//*********************************************************
typedef struct M3_CAN_FRAME
{
	uint32_t	uiId;
	uint8_t		ucLen;
	uint8_t		aucData[8];
	uint8_t		ucRecvFlag;
}T_M3_CAN_FRAME;

//*********************************************************
//                      全局变量申明
//*********************************************************

extern CAN_MSGOBJECT 		pMsg;
extern CAN_FILTEROBJECT 	pFilter;
extern CAN_RXMSGOBJECT 		rx_msg0;
extern CAN_RXMSGOBJECT 		rx_msg1;

extern uint8_t can_receive_count;						// can frame count
extern uint8_t can_receive_buffer[128];					// can data receive buffer

//*********************************************************
//                      函数申明
//*********************************************************
void CAN_Demo(void);
void CAN_Receive(void);
void CAN_Send(uint8_t * tx_buffer);
void CAN_Init(void);
void check_rx_buffer(void);



#endif /* CAN_H_ */
