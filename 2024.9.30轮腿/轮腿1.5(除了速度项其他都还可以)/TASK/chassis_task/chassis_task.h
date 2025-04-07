#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H	 

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "RemoteControl.h"

#include "get_judge_measure.h"
#include "gimbal_task.h"
#include "Ramp_Control.h"
#include "accelerometer_calibration.h"
#include "timer.h"
#include "Kalaman.h"


#define RC_deadline 4.0f

#define switch_up 1
#define switch_mid 3
#define switch_down 2

#define switch_left 1
#define switch_right 0

#define  RadtoTheat 							 180.f/3.1415926f
#define  TheattoRad  							 3.1415926f/180.f
#define  MPU_offect    6.1f
#define  pitch_offect    1.0f


#define wheel_theta_K    0.5f
#define wheel_v_x_K     0.5f

#define leg_theta_K    2.0f
#define leg_v_x_K     1.0f

#define LK_9025_R       0.0445f    //9025 v2   电机直径89mm       

#define all_Mg    90.0f     //整车重量  单位N    11.5*9.8
#define wheel_mg  11.76f                 //轮子重量 1.2*9.8


#define Chassis_task_Time    0.003f            //3ms,任务控制周期



#define  CHASSIS_RELAX           	  0//底盘无力模式
#define  CHASSIS_INIT           	  1
#define  CHASSIS_FOLLOW_GIMBAL  	  2//自动跟随云台
#define  CHASSIS_SEPARATE_GIMBAL    3//云台分离
#define  CHASSIS_DODGE_MODE         4//小陀螺
#define  CHASSIS_LEG_INIT_MODE      5//腿部初始化



typedef struct
{
	PidTypeDef leg_pid,right_leg_pid,turn_pid,theta_error_pid,roll_pid;
	float YD,YB,XD,XB,LBD,A0,B0,C0,XC,YC,L0,d_L0,d2_L0,last_L0,last_d_L0;
	float fai0,fai1,fai2,fai3,fai4,theta,d_theta,d2_theta,last_theta,last_d_theta;
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[2];

	float F0;
	float Tp;	
	float FN;//支持力	
	float LQR_K[12],u[12];	
	float wheel_T;
	float leg_T[2];
	float L0_set,last_L0_set;
	float v_set,x_set;
	float theta_error_T;
	float turn_T,ROLL_T;
	float L0_error,last_L0_error;
	float L0_out;
	float d_theta_filter;
	float recover_flag;  //起立标志位
	float leg_flag;        //离地标志位
	float v_error;
	
	
}vmc_leg;

extern float L0_P_out,L0_D_out;






void chassis_L_task(void);
void chassis_L_init(void);
void chassis_R_task(void);
void chassis_R_init(void);

void mecanum_calc(float vx, float vy, float vz, float speed[]);
float wheel_fabsf(float input);
int16_t rc_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
void chassis_power_limit_app(void);
void Super_power_ctrl(void);
float fp_deadline(float Value, float minValue, float maxValue);
#endif




