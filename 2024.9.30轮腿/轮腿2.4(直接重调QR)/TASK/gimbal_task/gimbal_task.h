#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H	

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "RemoteControl.h"
#include "arm_math.h"
#include "Chassis_task.h"
#include "TX2.h"


#define gimbal_ecd_mode 1
#define gimbal_imu_mode 2
#define gimbal_pc_mode 3


typedef struct
{
	PidTypeDef	speed_pid,ecd_pid,init_speed_pid,init_ecd_pid,auto_speed_pid,auto_ecd_pid,spin_ecd_pid,spin_speed_pid;
	float relative_angle;
	float absolute_angle;
	float last_absolute_angle;
	float angle_set;
	float current;
	float offset_ecd;
	float ecd_maxout,ecd_maxiout,speed_maxout,speed_maxiout;
	float last_set;
	float kf_out;
	float kf;
	float KT_out;
	
}gimbal_typedef;



void gimbal_task(void);
void gimbal_init(void);
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
int16_t Limit_mouse(int16_t input,int16_t min,int16_t max);
float fp_Limit_mouse(float input,float min,float max);
float get_relative_pos_pitch(float raw_ecd, float center_offset);

#endif




