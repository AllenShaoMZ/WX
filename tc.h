#include <os.h>

/*-----------------遥控寄存器----------------------*/

#define CMDQ_MAXLEN  32
#define ZSQ_MAXLEN   16
#define YWQ_MAXLEN   8

void    cmdEnqueue(CPU_INT08U * cmdCodeAddr);

//#pragma pack(1)
//typedef struct					//定义指令队列
//{
//    CPU_INT08U  addr;                                          
//    CPU_INT16U  cmd_code;
//}CMD_Q;

//#pragma pack()

