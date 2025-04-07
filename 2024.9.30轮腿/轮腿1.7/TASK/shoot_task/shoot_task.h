#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "sys.h"
#include "pid.h"
#include "RemoteControl.h"
#include "chassis_task.h"
#include "TX2.h"

#define fire_booster_30_set    3300

#define TIM5_OPEN_PWM    18600
#define TIM5_CLOSE_PWM   19670

extern int EE_Flag,RR_Flag,QQ_Flag;

void shoot_task(void);
void shoot_init(void);

typedef enum
{
	shoot_stop=0,
	shoot_one,
	shoot_fire,
	shoot_return
	
} shoot_mode_enum;        //���ģʽö��


typedef struct
{	
	float now_angle;								//��������λ��
	float angle_set;						//�����Ƕ�����
	float now_speed;	
	float speed_set;   					//�����ٶ�����

	float current;	
}booster_typedef;               //��������

typedef struct
{	
	float speed_set;
	float now_speed;
	
	float current;
}friction_typedef;              //Ħ����


typedef struct
{
	booster_typedef booster_t;           //������
	friction_typedef friction_left,friction_right;       //Ħ����
	shoot_mode_enum shoot_mode,last_shoot_mode;              //���ģʽ
	float rest_heat;   					//ʣ������

	
	
}shoot_typedef;

typedef struct
{
	PidTypeDef m2006_speed_pid,m2006_ecd_pid,left_friction_speed_pid,right_friction_speed_pid;

}shoot_pid_typedef;

#endif






