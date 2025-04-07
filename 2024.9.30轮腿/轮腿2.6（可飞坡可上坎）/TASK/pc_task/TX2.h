#ifndef __TX2_H_
#define __TX2_H_
	
#include "FreeRTOS.h"
#include "task.h"
#include "judgement_info.h"
#include "queue.h"
#include "stdio.h"
#include "string.h"
#include "IMUTASK.h"
#include "protocol.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "get_judge_measure.h"
#include "judgement_info.h"
#include "gimbal_task.h"
#include "RemoteControl.h"

extern int BB_Flag;


typedef __packed struct
{
    unsigned char head;
    float angle_pitch;
    float angle_yaw;
	 // float distance;
	  unsigned char shoot_flag;    //自动开火flag
	  unsigned char real_shoot_flag;		
		unsigned char target;     //是否识别到
		unsigned char start_flag;	    //自瞄是否开启
    unsigned char CRC8;
} PC_Ctrl_t;

//上位机数据转换共用体
typedef union
{
    PC_Ctrl_t PcDate;
    unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
} PC_Ctrl_Union_t;

typedef struct
{
    float yaw;
    float pitch;
    unsigned char mode;//模式 1：敌方小陀螺  3：视觉开火	9：能量机关风车   
    unsigned char color;//颜色
    unsigned char shoot_speed;//射速
		unsigned char kong;
    unsigned char CRC8;
} Send_Tx2_t;


void Send_Task(void *pvParameters);
void PC_receive_Task(void *pvParameters);
void Send_to_PC(USART_TypeDef* USARTx, Send_Tx2_t *TXmessage);

#endif


