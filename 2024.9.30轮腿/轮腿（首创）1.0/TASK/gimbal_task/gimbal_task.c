#include "gimbal_task.h"
#include "can.h"
#include "IMUTask.h"

extern motor_measure_t yaw_data,pitch_data;
extern  int input_mode;
extern PC_Ctrl_Union_t PcData;

gimbal_typedef yaw_motor,pitch_motor;
int gimbal_mode=0,gimbal_last_mode=0;
int Z_Flag=0,X_Flag=0,C_Flag=0;
int yaw_Flag=0,pitch_Flag=0;

int16_t Limit_mouse(int16_t input,int16_t min,int16_t max)
{
    if (input < min)
        return min;
    else if (input > max)
        return max;
    else
        return input;
}

float fp_Limit_mouse(float input,float min,float max)
{
    if (input < min)
        return min;
    else if (input > max)
        return max;
    else
        return input;
}

int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }

    return tmp;
}

void gimbal_init(void)
{
/**********************************反陀螺模式pid**************************************************/
		yaw_motor.spin_ecd_pid.Kp=12.0f;yaw_motor.spin_ecd_pid.Ki=0.1f;yaw_motor.spin_ecd_pid.Kd=0.0f;
		yaw_motor.spin_speed_pid.Kp=320.0f;yaw_motor.spin_speed_pid.Ki=0.0f;yaw_motor.spin_speed_pid.Kd=0.0f;
		yaw_motor.ecd_maxout=30000;yaw_motor.ecd_maxiout=30;
		yaw_motor.speed_maxout=30000,yaw_motor.speed_maxiout=0;	
//		pitch_motor.spin_ecd_pid.Kp=20.0f;pitch_motor.spin_ecd_pid.Ki=0.1f;pitch_motor.spin_ecd_pid.Kd=0.0f;
//		pitch_motor.spin_speed_pid.Kp=280.0f;pitch_motor.spin_speed_pid.Ki=0.0f  ;pitch_motor.spin_speed_pid.Kd=0.0f;	
		pitch_motor.spin_ecd_pid.Kp=14.0f;pitch_motor.spin_ecd_pid.Ki=0.1f;pitch_motor.spin_ecd_pid.Kd=0.0f;
		pitch_motor.spin_speed_pid.Kp=300.0f;pitch_motor.spin_speed_pid.Ki=0.0f  ;pitch_motor.spin_speed_pid.Kd=20.0f;		
		pitch_motor.ecd_maxout=30000;pitch_motor.ecd_maxiout=30;	
		pitch_motor.speed_maxout=30000;pitch_motor.speed_maxiout=0;		
/**********************************打符模式pid**************************************************/
		yaw_motor.auto_ecd_pid.Kp=15.0f;yaw_motor.auto_ecd_pid.Ki=0.1f;yaw_motor.auto_ecd_pid.Kd=10.0f;
		yaw_motor.auto_speed_pid.Kp=300.0f;yaw_motor.auto_speed_pid.Ki=0.0f;yaw_motor.auto_speed_pid.Kd=70.0f;
		yaw_motor.ecd_maxout=30000;yaw_motor.ecd_maxiout=30;
		yaw_motor.speed_maxout=30000,yaw_motor.speed_maxiout=0;	
//		pitch_motor.auto_ecd_pid.Kp=18.0f;pitch_motor.auto_ecd_pid.Ki=0.1f;pitch_motor.auto_ecd_pid.Kd=0.0f;
//		pitch_motor.auto_speed_pid.Kp=300.0f;pitch_motor.auto_speed_pid.Ki=0.0f  ;pitch_motor.auto_speed_pid.Kd=0.0f;	
		pitch_motor.auto_ecd_pid.Kp=14.0f;pitch_motor.auto_ecd_pid.Ki=0.1f;pitch_motor.auto_ecd_pid.Kd=10.0f;
		pitch_motor.auto_speed_pid.Kp=300.0f;pitch_motor.auto_speed_pid.Ki=0.0f  ;pitch_motor.auto_speed_pid.Kd=200.0f;		
		pitch_motor.ecd_maxout=30000;pitch_motor.ecd_maxiout=30;	
		pitch_motor.speed_maxout=30000;pitch_motor.speed_maxiout=0;	
/**********************************手动模式pid**************************************************/
		yaw_motor.ecd_pid.Kp=0.0f;yaw_motor.ecd_pid.Ki=0.0f;yaw_motor.ecd_pid.Kd=0.0f;
		yaw_motor.speed_pid.Kp=0.0f;yaw_motor.speed_pid.Ki=0.0f;yaw_motor.speed_pid.Kd=0.0f;
		yaw_motor.ecd_maxout=30000;yaw_motor.ecd_maxiout=30;
		yaw_motor.speed_maxout=30000,yaw_motor.speed_maxiout=0;	

		pitch_motor.ecd_pid.Kp=0.0f;pitch_motor.ecd_pid.Ki=0.0f;pitch_motor.ecd_pid.Kd=0.0f;
		pitch_motor.speed_pid.Kp=0.0f;pitch_motor.speed_pid.Ki=0.0f;pitch_motor.speed_pid.Kd=0.0f;	
		pitch_motor.ecd_maxout=30000;pitch_motor.ecd_maxiout=30;	
		pitch_motor.speed_maxout=30000;pitch_motor.speed_maxiout=0;
/**********************************归中模式pid**************************************************/
		yaw_motor.init_ecd_pid.Kp=0.0f;yaw_motor.init_ecd_pid.Ki=0.0f;yaw_motor.init_ecd_pid.Kd=0.0f;
		yaw_motor.init_speed_pid.Kp=0.0f;yaw_motor.init_speed_pid.Ki=0.0f;yaw_motor.init_speed_pid.Kd=0.0f;
		pitch_motor.init_ecd_pid.Kp=0.0f;pitch_motor.init_ecd_pid.Ki=0.0f;pitch_motor.init_ecd_pid.Kd=0.0f;
		pitch_motor.init_speed_pid.Kp=0.0f;pitch_motor.init_speed_pid.Ki=0.0f;pitch_motor.init_speed_pid.Kd=0.0f;
/**********************************手动模式pid**************************************************/	
		pid_init(&yaw_motor.ecd_pid,PID_POSITION,yaw_motor.ecd_pid.Kp,yaw_motor.ecd_pid.Ki,yaw_motor.ecd_pid.Kd,yaw_motor.ecd_maxout,yaw_motor.ecd_maxiout);
		pid_init(&yaw_motor.speed_pid,PID_POSITION,yaw_motor.speed_pid.Kp,yaw_motor.speed_pid.Ki,yaw_motor.speed_pid.Kd,yaw_motor.speed_maxout,yaw_motor.speed_maxiout);
	
		pid_init(&pitch_motor.ecd_pid,PID_POSITION,pitch_motor.ecd_pid.Kp,pitch_motor.ecd_pid.Ki,pitch_motor.ecd_pid.Kd,pitch_motor.ecd_maxout,pitch_motor.ecd_maxiout);
		pid_init(&pitch_motor.speed_pid,PID_POSITION,pitch_motor.speed_pid.Kp,pitch_motor.speed_pid.Ki,pitch_motor.speed_pid.Kd,pitch_motor.speed_maxout,pitch_motor.speed_maxiout);
/**********************************归中模式pid**************************************************/	
		pid_init(&yaw_motor.init_ecd_pid,PID_POSITION,yaw_motor.init_ecd_pid.Kp,yaw_motor.init_ecd_pid.Ki,yaw_motor.init_ecd_pid.Kd,yaw_motor.ecd_maxout,0);
		pid_init(&yaw_motor.init_speed_pid,PID_POSITION,yaw_motor.init_speed_pid.Kp,yaw_motor.init_speed_pid.Ki,yaw_motor.init_speed_pid.Kd,yaw_motor.speed_maxout,0);
		
		pid_init(&pitch_motor.init_ecd_pid,PID_POSITION,pitch_motor.init_ecd_pid.Kp,pitch_motor.init_ecd_pid.Ki,pitch_motor.init_ecd_pid.Kd,pitch_motor.ecd_maxout,0);
		pid_init(&pitch_motor.init_speed_pid,PID_POSITION,pitch_motor.init_speed_pid.Kp,pitch_motor.init_speed_pid.Ki,pitch_motor.init_speed_pid.Kd,pitch_motor.speed_maxout,0);		
/**********************************打符模式pid**************************************************/	
		pid_init(&yaw_motor.auto_ecd_pid,PID_POSITION,yaw_motor.auto_ecd_pid.Kp,yaw_motor.auto_ecd_pid.Ki,yaw_motor.auto_ecd_pid.Kd,500.0f,30);
		pid_init(&yaw_motor.auto_speed_pid,PID_POSITION,yaw_motor.auto_speed_pid.Kp,yaw_motor.auto_speed_pid.Ki,yaw_motor.auto_speed_pid.Kd,yaw_motor.speed_maxout,yaw_motor.speed_maxiout);
	
		pid_init(&pitch_motor.auto_ecd_pid,PID_POSITION,pitch_motor.auto_ecd_pid.Kp,pitch_motor.auto_ecd_pid.Ki,pitch_motor.auto_ecd_pid.Kd,500.0f,30);
		pid_init(&pitch_motor.auto_speed_pid,PID_POSITION,pitch_motor.auto_speed_pid.Kp,pitch_motor.auto_speed_pid.Ki,pitch_motor.auto_speed_pid.Kd,pitch_motor.speed_maxout,pitch_motor.speed_maxiout);
		
/**********************************反陀螺模式pid**************************************************/	
		pid_init(&yaw_motor.spin_ecd_pid,PID_POSITION,yaw_motor.spin_ecd_pid.Kp,yaw_motor.spin_ecd_pid.Ki,yaw_motor.spin_ecd_pid.Kd,500,30);
		pid_init(&yaw_motor.spin_speed_pid,PID_POSITION,yaw_motor.spin_speed_pid.Kp,yaw_motor.spin_speed_pid.Ki,yaw_motor.spin_speed_pid.Kd,yaw_motor.speed_maxout,yaw_motor.speed_maxiout);
	
		pid_init(&pitch_motor.spin_ecd_pid,PID_POSITION,pitch_motor.spin_ecd_pid.Kp,pitch_motor.spin_ecd_pid.Ki,pitch_motor.spin_ecd_pid.Kd,400,30);
		pid_init(&pitch_motor.spin_speed_pid,PID_POSITION,pitch_motor.spin_speed_pid.Kp,pitch_motor.spin_speed_pid.Ki,pitch_motor.spin_speed_pid.Kd,pitch_motor.speed_maxout,pitch_motor.speed_maxiout);		
		
}

void gimbal_task(void)
{
			yaw_motor.offset_ecd=2699;               //初始化归中值
			pitch_motor.offset_ecd=2658;
			rc_ctrl.rc.ch[2]=rc_deadline(rc_ctrl.rc.ch[2],-4,4);
			rc_ctrl.rc.ch[3]=rc_deadline(rc_ctrl.rc.ch[3],-4,4);	
			yaw_motor.relative_angle=(360.0f/8192.0f)*get_relative_pos(yaw_data.ecd,yaw_motor.offset_ecd);
			yaw_motor.absolute_angle=-Angular_Handler.YAW;	
			pitch_motor.absolute_angle=Angular_Handler.ROLL;               //板子横放用ROLL  竖直放用PITCH
			if(pitch_data.ecd>=2200&&pitch_data.ecd<=2850)
			{
				pitch_motor.KT_out=-5.4492f*pitch_data.ecd+15728.0f;
			}
			else
			{
				pitch_motor.KT_out=0;
			}
			if(input_mode==stop_mode)//双下掉电
			{
					gimbal_last_mode=gimbal_mode=0;
					yaw_motor.current=0;	
					pitch_motor.current=0;	
			}
/**********************************************遥控器模式****************************************************/			
			else if(input_mode==remote_mode)
			{
				if(rc_ctrl.rc.s[switch_left]==switch_down&&rc_ctrl.rc.s[switch_right]==switch_mid)  //左下右中，云台初始化	
				{
						gimbal_last_mode=gimbal_mode;
						gimbal_mode=gimbal_ecd_mode;
						yaw_motor.last_absolute_angle=yaw_motor.absolute_angle;				
						PID_Calc(&yaw_motor.init_ecd_pid,yaw_motor.relative_angle,0);
						PID_Calc(&yaw_motor.init_speed_pid,yaw_data.speed_rpm,yaw_motor.init_ecd_pid.out);
						PID_Calc(&pitch_motor.init_ecd_pid,pitch_motor.absolute_angle,0);
						PID_Calc(&pitch_motor.init_speed_pid,pitch_data.speed_rpm,pitch_motor.init_ecd_pid.out);
						yaw_motor.current=yaw_motor.init_speed_pid.out;
						pitch_motor.current=pitch_motor.init_speed_pid.out;					
				}
				else if((rc_ctrl.rc.s[switch_left]==switch_mid&&rc_ctrl.rc.s[switch_right]==switch_mid)||(rc_ctrl.rc.s[switch_left]==switch_up&&rc_ctrl.rc.s[switch_right]==switch_mid))  //双中，手动控制
				{
						gimbal_last_mode=gimbal_mode;
						gimbal_mode=gimbal_imu_mode;
						if(gimbal_last_mode==gimbal_ecd_mode)
						{
							yaw_motor.angle_set=yaw_motor.last_absolute_angle;		
						}
						yaw_motor.last_absolute_angle=yaw_motor.absolute_angle;		
						yaw_motor.last_set=yaw_motor.angle_set;
						pitch_motor.last_set=pitch_motor.angle_set;						
						yaw_motor.angle_set-=rc_ctrl.rc.ch[2]*0.0002f;
						pitch_motor.angle_set+=rc_ctrl.rc.ch[3]*0.0002f;
						pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
						PID_Calc(&pitch_motor.ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
						PID_Calc(&pitch_motor.speed_pid,Angular_Handler.V_X,pitch_motor.ecd_pid.out);								
						PID_Calc(&yaw_motor.ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
						PID_Calc(&yaw_motor.speed_pid,Angular_Handler.V_Z,yaw_motor.ecd_pid.out);	
						pitch_motor.current=pitch_motor.speed_pid.out;		
						yaw_motor.current=yaw_motor.speed_pid.out;					
				}
			}
/***************************************************键盘模式******************************************************/		
			else if(input_mode==pc_mode)
			{
				gimbal_last_mode=gimbal_mode;
				gimbal_mode=gimbal_pc_mode;
				if(gimbal_last_mode==gimbal_ecd_mode||gimbal_last_mode==gimbal_imu_mode)
				{
					yaw_motor.angle_set=yaw_motor.last_absolute_angle;
				}
				yaw_motor.last_set=yaw_motor.angle_set;
				pitch_motor.last_set=pitch_motor.angle_set;
				//pitch_motor.KT_out=-6.176f*pitch_data.ecd+17766.0f;					//重力补偿 线性回归模型
				if (IF_KEY_PRESSED_Z&&!Z_Flag&&rc_ctrl.key.v)
        {
            Z_Flag = 1;
        }				
				if(Z_Flag==1&&!IF_KEY_PRESSED_Z)
				{
					yaw_motor.angle_set=yaw_motor.angle_set+90.0f;
					Z_Flag=0;
				}	
        if (IF_KEY_PRESSED_X&&!X_Flag&&rc_ctrl.key.v)
        {
            X_Flag = 1;
        }				
				if(X_Flag==1&&!IF_KEY_PRESSED_X)
				{
					yaw_motor.angle_set=yaw_motor.angle_set+180.0f;
					X_Flag=0;
				}	
        if (IF_KEY_PRESSED_C&&!C_Flag&&rc_ctrl.key.v)
        {
            C_Flag = 1;
        }				
				if(C_Flag==1&&!IF_KEY_PRESSED_C)
				{
					yaw_motor.angle_set=yaw_motor.angle_set-90.0f;
					C_Flag=0;
				}
				if(PcData.PcDate.angle_yaw<25.0f&&PcData.PcDate.angle_yaw>-25.0f)
				{
					yaw_Flag=1;
				}
				else
				{
					yaw_Flag=0;
				}
				if(PcData.PcDate.angle_pitch<20.0f&&PcData.PcDate.angle_pitch>-20.0f)
				{
					pitch_Flag=1;
				}
				else
				{
					pitch_Flag=0;
				}				
//				if(GG_Flag==1&&rc_ctrl.mouse.press_r==1&&PcData.PcDate.target==1)
//				{	
//					yaw_motor.kf=0;
//					pitch_motor.kf=0;//400
//					yaw_motor.angle_set=yaw_motor.absolute_angle-rc_ctrl.mouse.x*0.0012f+PcData.PcDate.angle_yaw*yaw_Flag;
//					pitch_motor.angle_set=pitch_motor.absolute_angle-rc_ctrl.mouse.y*0.00012f-PcData.PcDate.angle_pitch*pitch_Flag;
//					pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
//					PID_Calc_pitch(&pitch_motor.auto_ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
//					PID_Calc(&pitch_motor.auto_speed_pid,Angular_Handler.V_X,pitch_motor.auto_ecd_pid.out);								
//					PID_Calc_yaw(&yaw_motor.auto_ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
//					PID_Calc(&yaw_motor.auto_speed_pid,Angular_Handler.V_Z,yaw_motor.auto_ecd_pid.out);
//					yaw_motor.kf_out=(yaw_motor.angle_set-yaw_motor.last_set)*yaw_motor.kf;
//					pitch_motor.kf_out=(pitch_motor.angle_set-pitch_motor.last_set)*pitch_motor.kf;
//					pitch_motor.current=pitch_motor.auto_speed_pid.out+pitch_motor.kf_out+pitch_motor.KT_out;					
//					yaw_motor.current=yaw_motor.auto_speed_pid.out+yaw_motor.kf_out;							
//				}
//				else if(GG_Flag==0&&rc_ctrl.mouse.press_r==1&&PcData.PcDate.target==1)
//				{
//					yaw_motor.kf=0;
//					pitch_motor.kf=0;//400					
//					yaw_motor.angle_set=yaw_motor.absolute_angle-rc_ctrl.mouse.x*0.0012f+PcData.PcDate.angle_yaw*yaw_Flag;
//					pitch_motor.angle_set=pitch_motor.absolute_angle-rc_ctrl.mouse.y*0.00012f-PcData.PcDate.angle_pitch*pitch_Flag;
//					pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
//					PID_Calc_pitch_spin(&pitch_motor.spin_ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
//					PID_Calc(&pitch_motor.spin_speed_pid,Angular_Handler.V_X,pitch_motor.spin_ecd_pid.out);								
//					PID_Calc_yaw_spin(&yaw_motor.spin_ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
//					PID_Calc(&yaw_motor.spin_speed_pid,Angular_Handler.V_Z,yaw_motor.spin_ecd_pid.out);
//					yaw_motor.kf_out=(yaw_motor.angle_set-yaw_motor.last_set)*yaw_motor.kf;
//					pitch_motor.kf_out=(pitch_motor.angle_set-pitch_motor.last_set)*pitch_motor.kf;
//					pitch_motor.current=pitch_motor.spin_speed_pid.out+pitch_motor.kf_out+pitch_motor.KT_out;						
//					yaw_motor.current=yaw_motor.spin_speed_pid.out+yaw_motor.kf_out;						
//				}
//				else
//				{				
//					yaw_motor.angle_set=yaw_motor.angle_set-rc_ctrl.mouse.x*0.0012f;
//					pitch_motor.angle_set=pitch_motor.angle_set-rc_ctrl.mouse.y*0.0012f;				
//					pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
//					PID_Calc(&pitch_motor.ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
//					PID_Calc(&pitch_motor.speed_pid,Angular_Handler.V_X,pitch_motor.ecd_pid.out);									
//					PID_Calc(&yaw_motor.ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
//					PID_Calc(&yaw_motor.speed_pid,Angular_Handler.V_Z,yaw_motor.ecd_pid.out);					
//					pitch_motor.current=pitch_motor.speed_pid.out+pitch_motor.KT_out;				
//					yaw_motor.current=yaw_motor.speed_pid.out+yaw_motor.KT_out;					
//				}
			}
			else if(input_mode==aim_mode)
			{
				if(PcData.PcDate.angle_yaw<25.0f&&PcData.PcDate.angle_yaw>-25.0f)
				{
					yaw_Flag=1;
				}
				else
				{
					yaw_Flag=0;
				}
				if(PcData.PcDate.angle_pitch<25.0f&&PcData.PcDate.angle_pitch>-25.0f)
				{
					pitch_Flag=1;
				}
				else
				{
					pitch_Flag=0;
				}						
				yaw_motor.angle_set=yaw_motor.absolute_angle-rc_ctrl.mouse.x*0.0012f+PcData.PcDate.angle_yaw*yaw_Flag;
				pitch_motor.angle_set=pitch_motor.absolute_angle-rc_ctrl.mouse.y*0.00012f-PcData.PcDate.angle_pitch*pitch_Flag;
				pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
				PID_Calc_pitch(&pitch_motor.auto_ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
				PID_Calc(&pitch_motor.auto_speed_pid,Angular_Handler.V_X,pitch_motor.auto_ecd_pid.out);								
				PID_Calc_yaw(&yaw_motor.auto_ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
				PID_Calc(&yaw_motor.auto_speed_pid,Angular_Handler.V_Z,yaw_motor.auto_ecd_pid.out);
				yaw_motor.kf_out=(yaw_motor.angle_set-yaw_motor.last_set)*yaw_motor.kf;
				pitch_motor.kf_out=(pitch_motor.angle_set-pitch_motor.last_set)*pitch_motor.kf;
				pitch_motor.current=pitch_motor.auto_speed_pid.out+pitch_motor.kf_out+pitch_motor.KT_out;						
				yaw_motor.current=yaw_motor.auto_speed_pid.out+yaw_motor.kf_out;				
			}
			else if(input_mode==spin_mode)
			{
				if(PcData.PcDate.angle_yaw<25.0f&&PcData.PcDate.angle_yaw>-25.0f)
				{
					yaw_Flag=1;
				}
				else
				{
					yaw_Flag=0;
				}
				if(PcData.PcDate.angle_pitch<25.0f&&PcData.PcDate.angle_pitch>-25.0f)
				{
					pitch_Flag=1;	
				}
				else
				{
					pitch_Flag=0;
				}						
					yaw_motor.kf=0;
					pitch_motor.kf=0;//400					
					yaw_motor.angle_set=yaw_motor.absolute_angle-rc_ctrl.mouse.x*0.0012f+PcData.PcDate.angle_yaw*yaw_Flag;
					pitch_motor.angle_set=pitch_motor.absolute_angle-rc_ctrl.mouse.y*0.00012f-PcData.PcDate.angle_pitch*pitch_Flag;
					pitch_motor.angle_set=fp_Limit_mouse(pitch_motor.angle_set,-24.07f,29.05f);
					PID_Calc_pitch_spin(&pitch_motor.spin_ecd_pid,pitch_motor.absolute_angle,pitch_motor.angle_set);
					PID_Calc(&pitch_motor.spin_speed_pid,Angular_Handler.V_X,pitch_motor.spin_ecd_pid.out);								
					PID_Calc_yaw_spin(&yaw_motor.spin_ecd_pid,yaw_motor.absolute_angle,yaw_motor.angle_set);
					PID_Calc(&yaw_motor.spin_speed_pid,Angular_Handler.V_Z,yaw_motor.spin_ecd_pid.out);
					yaw_motor.kf_out=(yaw_motor.angle_set-yaw_motor.last_set)*yaw_motor.kf;
					pitch_motor.kf_out=(pitch_motor.angle_set-pitch_motor.last_set)*pitch_motor.kf;
					pitch_motor.current=pitch_motor.spin_speed_pid.out+pitch_motor.kf_out+pitch_motor.KT_out;						
					yaw_motor.current=yaw_motor.spin_speed_pid.out+yaw_motor.kf_out;				
			}			
			pitch_motor.current=fp_Limit_mouse(pitch_motor.current,-30000.0f,30000.0f);
			yaw_motor.current=fp_Limit_mouse(yaw_motor.current,-30000.0f,30000.0f);					
			CAN1_SetMsg_6020_gimbal(yaw_motor.current);
			CAN2_SetMsg_6020(0,0,pitch_motor.current,0);
}





