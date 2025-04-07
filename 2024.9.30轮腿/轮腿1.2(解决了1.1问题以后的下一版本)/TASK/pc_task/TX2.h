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
	  unsigned char shoot_flag;    //�Զ�����flag
	  unsigned char real_shoot_flag;		
		unsigned char target;     //�Ƿ�ʶ��
		unsigned char start_flag;	    //�����Ƿ���
    unsigned char CRC8;
} PC_Ctrl_t;

//��λ������ת��������
typedef union
{
    PC_Ctrl_t PcDate;
    unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
} PC_Ctrl_Union_t;

typedef struct
{
    float yaw;
    float pitch;
    unsigned char mode;//ģʽ 1���з�С����  3���Ӿ�����	9���������ط糵   
    unsigned char color;//��ɫ
    unsigned char shoot_speed;//����
		unsigned char kong;
    unsigned char CRC8;
} Send_Tx2_t;


void Send_Task(void *pvParameters);
void PC_receive_Task(void *pvParameters);
void Send_to_PC(USART_TypeDef* USARTx, Send_Tx2_t *TXmessage);

#endif


