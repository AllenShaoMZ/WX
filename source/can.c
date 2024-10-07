/*
 * can.c
 *
 *  Created on: 2019?11?29?
 *      Author: Administrator
 */

#include "can.h" 
#include "FPGAUart.h"

CAN_MSGOBJECT 		pMsg;
CAN_FILTEROBJECT 	pFilter;
CAN_RXMSGOBJECT 	rx_msg0;
CAN_RXMSGOBJECT 	rx_msg1;

uint8_t can_receive_count = 0;				// can frame count
uint8_t can_receive_buffer[128];			// can data receive buffer

extern void delay_ms(uint32_t ms);
extern	void CAN_IRQHandler(void);

/*CAN中断服务函数*/
void CAN_IRQHandler(void)
{
	volatile uint32_t read_irq_state = 0;

	read_irq_state = MSS_CAN_get_int_status(&g_can0);

	if(0 != (read_irq_state | CAN_INT_RX_MSG))
	{
		MSS_CAN_clear_int_status(&g_can0, CAN_INT_RX_MSG);		// clear rx int
		CAN_Receive();											// can receive isr
	}
}


void CAN_Init(void)
{
	MSS_CAN_init(&g_can0,										// 16M APB 500K
				 CAN_SET_BITRATE(3)|CAN_SET_TSEG1(3)|CAN_SET_TSEG2(2)|CAN_THREE_SAMPLES|CAN_LITTLE_ENDIAN,		// tseg new
				 (PCAN_CONFIG_REG)0,
				 0,
				 0);

	MSS_CAN_set_mode(&g_can0, CANOP_MODE_NORMAL); 				// normal CAN
	MSS_CAN_start(&g_can0);

	
	rx_msg0.ID = 0x200;
    rx_msg0.DATAHIGH = 0u;
    rx_msg0.DATALOW = 0u;

    rx_msg0.AMR.L = 0xFFFFFFFF;
    rx_msg0.ACR.L = 0x00000000;
    rx_msg0.AMR_D = 0xFFFFFFFF;
    rx_msg0.ACR_D = 0x00000000;
    rx_msg0.RXB.DLC = 8u;
    rx_msg0.RXB.IDE = 1u;
	
 	MSS_CAN_config_buffer_n(&g_can0, 0, &rx_msg0);

	MSS_CAN_set_int_ebl(&g_can0,  ( CAN_INT_GLOBAL | CAN_INT_RX_MSG ) );
}

void CAN_Send_OneFrame(uint8_t * tx_buffer, uint8_t frame_len)
{
	unsigned int i;

	for( i = 0; i < frame_len; i++ )							// RS SEND & RECEIVE  --  vTaskDelay --
	{
		pMsg.DATA[i] = tx_buffer[i];
	}
														// CAN ID
	pMsg.ID  = 0x550000;
	pMsg.IDE = 0;
	pMsg.RTR = 0;
	pMsg.DLC = frame_len;										// DLC 4byte

	pMsg.ID = MSS_CAN_set_id(&pMsg);					// set can ID

	can_receive_count = 0;								// frame count clear
	//MSS_CAN_send_message(&g_can0, &pMsg);				// send can msg
	MSS_CAN_send_message_n(&g_can0, 0, &pMsg);			// send can msg
}

void CAN_Send_Array(uint8_t * tx_buffer, uint8_t Tlen)
{
	uint8_t frame_num;
	uint8_t tx_k;

	if(Tlen==0)
	{
		return;
	}

	frame_num = Tlen / 8;
	if( (Tlen % 8)!=0 )
	{
		frame_num = frame_num + 1;
	}
	for(tx_k=0; tx_k<frame_num ;tx_k++)
	{
		if(tx_k==(frame_num-1)) //????
		{
			CAN_Send_OneFrame( &tx_buffer[tx_k*8], Tlen-(frame_num-1)*8 );
		}
		else
		{
			CAN_Send_OneFrame( &tx_buffer[tx_k*8],8);
		}
		//????
		delay_1ms(5);
	}
}



void CAN_Send(uint8_t * tx_buffer)
{
	unsigned int i;

	for( i = 0; i < 8; i++ )							// RS SEND & RECEIVE  --  vTaskDelay --
	{
		pMsg.DATA[i] = tx_buffer[i];
	}
														// CAN ID
	pMsg.ID  = 0x550000;
	pMsg.IDE = 1;
	pMsg.RTR = 0;
	pMsg.DLC = 8;										// DLC 4byte

	pMsg.ID = MSS_CAN_set_id(&pMsg);					// set can ID

	can_receive_count = 0;								// frame count clear
	//MSS_CAN_send_message(&g_can0, &pMsg);				// send can msg
	MSS_CAN_send_message_n(&g_can0, 0, &pMsg);			// send can msg
}


void CAN_Receive(void)
{
	uint8_t loop_count;
	uint32_t return_status = 0;
	CAN_MSGOBJECT can_rx_buffer;

	return_status = MSS_CAN_get_rx_buffer_status(&g_can0);
	if((return_status & 0x01) == 0x01)
	{
		MSS_CAN_get_message_n(&g_can0, 0, &can_rx_buffer);

		for(loop_count = 0; loop_count < can_rx_buffer.DLC; loop_count++)
		{
			can_receive_buffer[loop_count] = can_rx_buffer.DATA[loop_count];
		}
				
		can_receive_count++;
	}
	if((return_status & 0x02) == 0x02)
	{
		MSS_CAN_get_message_n(&g_can0, 1, &can_rx_buffer);

		
		for(loop_count = 0; loop_count < can_rx_buffer.DLC; loop_count++)
		{
			can_receive_buffer[loop_count] = can_rx_buffer.DATA[loop_count];
		}

		can_receive_count++;
	}
}

void CAN_Demo(void)
{
	unsigned int i;
	unsigned char tx_buffer[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

	for( i = 0; i < 8; i++ )								// RS SEND & RECEIVE  --  vTaskDelay --
	{
		pMsg.DATA[i] = tx_buffer[i];
	}
															// CAN ID
	pMsg.ID = 0x550000;
	pMsg.IDE = 0;
	pMsg.RTR = 0;
	pMsg.DLC = 8;											// DLC 4byte

	pMsg.ID = MSS_CAN_set_id(&pMsg);						// set can ID

	can_receive_count = 0;									// frame count clear
	//MSS_CAN_send_message(&g_can0, &pMsg);	// send can msg
	MSS_CAN_send_message_n(&g_can0, 0, &pMsg);
}

