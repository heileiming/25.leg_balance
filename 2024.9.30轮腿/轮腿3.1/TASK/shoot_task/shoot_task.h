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
	
} shoot_mode_enum;        //射击模式枚举


typedef struct
{	
	float now_angle;								//拨盘现在位置
	float angle_set;						//单发角度设置
	float now_speed;	
	float speed_set;   					//连发速度设置

	float current;	
}booster_typedef;               //拨弹机构

typedef struct
{	
	float speed_set;
	float now_speed;
	
	float current;
}friction_typedef;              //摩擦轮


typedef struct
{
	booster_typedef booster_t;           //拨弹轮
	friction_typedef friction_left,friction_right;       //摩擦轮
	shoot_mode_enum shoot_mode,last_shoot_mode;              //射击模式
	float rest_heat;   					//剩余热量

	
	
}shoot_typedef;

typedef struct
{
	PidTypeDef m2006_speed_pid,m2006_ecd_pid,left_friction_speed_pid,right_friction_speed_pid;

}shoot_pid_typedef;

#endif






