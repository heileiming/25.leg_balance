#include "shoot_task.h"
#include "can.h"

extern motor_measure_t m2006_data,friction_left_data,friction_right_data;
extern ext_power_heat_data_t	PowerHeatData;
extern ext_game_robot_status_t	GameRobotStat;
extern ext_shoot_data_t	ShootData;             //测速数据
extern projectile_allowance_t           pillData;  //弹丸数据

extern  int input_mode;
extern PC_Ctrl_Union_t PcData;


float fire_friction_30MS_SET=7400.0f;

int32_t if_shoot_time=0;  		//视觉开火发1的time
float last_shoot_speed;      		  //最近一次的射速
float average_shoot_speed=0; 						   //已发射弹的平均射速
float heat_limit;
int EE_Flag=0,RR_Flag=0,QQ_Flag=0;
int left_mouse_flag=0,left_mouse_last_flag=0;
float shoot_error=0;
uint16_t left_mouse_time=0;
shoot_typedef pc_shoot,remote_shoot;
shoot_pid_typedef shoot_pid;

float shoot_speed_record[10];
//				if(IF_KEY_PRESSED_R&&!R_Flag&&rc_ctrl.key.v)  //弹仓
//				{
//						R_Flag = 1;
//				}				
//				if(R_Flag==1&&!IF_KEY_PRESSED_R)
//				{
//						RR_Flag++;
//						RR_Flag%=2;
//						R_Flag=0;
//				}		
//				if(RR_Flag==1)
//				{
//						TIM_SetCompare1(TIM5, TIM5_OPEN_PWM);
//				}
//				else if(RR_Flag==0)
//				{
//						TIM_SetCompare1(TIM5, TIM5_CLOSE_PWM);
//				}
void shoot_init()
{
		pc_shoot.friction_left.speed_set=0;
		pc_shoot.friction_right.speed_set=0;
	
		shoot_pid.m2006_ecd_pid.Kp=100.0f;     //23发空一发
		shoot_pid.m2006_ecd_pid.Ki=0;
		shoot_pid.m2006_ecd_pid.Kd=0;
		shoot_pid.m2006_speed_pid.Kp=15.0f;
		shoot_pid.m2006_speed_pid.Ki=0;
		shoot_pid.m2006_speed_pid.Kd=0;		
	
		shoot_pid.left_friction_speed_pid.Kp=10.0f;
		shoot_pid.left_friction_speed_pid.Ki=0;
		shoot_pid.left_friction_speed_pid.Kd=0;	

		shoot_pid.right_friction_speed_pid.Kp=10.0f;
		shoot_pid.right_friction_speed_pid.Ki=0;
		shoot_pid.right_friction_speed_pid.Kd=0;	

		pid_init(&shoot_pid.m2006_ecd_pid,PID_POSITION,shoot_pid.m2006_ecd_pid.Kp,shoot_pid.m2006_ecd_pid.Ki,shoot_pid.m2006_ecd_pid.Kd,8000,0);
		pid_init(&shoot_pid.m2006_speed_pid,PID_POSITION,shoot_pid.m2006_speed_pid.Kp,shoot_pid.m2006_speed_pid.Ki,shoot_pid.m2006_speed_pid.Kd,8000,0);
	
		pid_init(&shoot_pid.left_friction_speed_pid,PID_POSITION,shoot_pid.left_friction_speed_pid.Kp,shoot_pid.left_friction_speed_pid.Ki,shoot_pid.left_friction_speed_pid.Kd,15000,0);	
		pid_init(&shoot_pid.right_friction_speed_pid,PID_POSITION,shoot_pid.right_friction_speed_pid.Kp,shoot_pid.right_friction_speed_pid.Ki,shoot_pid.right_friction_speed_pid.Kd,15000,0);	
}

void shoot_task()
{
		rc_deadline(rc_ctrl.rc.ch[4],-4,4);	
		pc_shoot.booster_t.now_angle=m2006_data.all_ecd*(360.0f/8191.0f/36.0f);
		pc_shoot.booster_t.now_speed=m2006_data.speed_rpm;
		pc_shoot.friction_left.now_speed=friction_left_data.speed_rpm;
		pc_shoot.friction_right.now_speed=friction_right_data.speed_rpm;	
		pc_shoot.rest_heat=GameRobotStat.shooter_barrel_heat_limit-PowerHeatData.shooter_17mm_1_barrel_heat;
		if((last_shoot_speed!=ShootData.initial_speed)&&ShootData.initial_speed>21.0f)
		{
			shoot_speed_record[0]=shoot_speed_record[1];
			shoot_speed_record[1]=shoot_speed_record[2];
			shoot_speed_record[2]=shoot_speed_record[3];
			shoot_speed_record[3]=shoot_speed_record[4];
			shoot_speed_record[4]=ShootData.initial_speed;
			average_shoot_speed=(shoot_speed_record[0]+shoot_speed_record[1]+shoot_speed_record[2]+shoot_speed_record[3]
					+shoot_speed_record[4])/5.0f;
//			shoot_speed_record[5]=shoot_speed_record[6];
//			shoot_speed_record[6]=shoot_speed_record[7];
//			shoot_speed_record[7]=shoot_speed_record[8];
//			shoot_speed_record[8]=shoot_speed_record[9];
//			shoot_speed_record[9]=ShootData.initial_speed;
//			average_shoot_speed=(shoot_speed_record[0]+shoot_speed_record[1]+shoot_speed_record[2]+shoot_speed_record[3]
//					+shoot_speed_record[4]+shoot_speed_record[5]+shoot_speed_record[6]
//					+shoot_speed_record[7]+shoot_speed_record[8]+shoot_speed_record[9])/10.0f;
		}
		if(input_mode==stop_mode)//双下掉电
		{
				EE_Flag=0;
				pc_shoot.friction_left.speed_set=0;
				pc_shoot.friction_right.speed_set=0;	
			
				pc_shoot.shoot_mode=shoot_stop;
				pc_shoot.booster_t.current=0;
				pc_shoot.friction_left.current=0;
				pc_shoot.friction_right.current=0;
				CAN2_SetMsg_shoot(0,0,0);
		}
/**********************************************遥控器模式****************************************************/			
		else if(input_mode==remote_mode)
		{
				if(rc_ctrl.rc.s[switch_left]==switch_mid&&rc_ctrl.rc.s[switch_right]==switch_mid)
				{
					if(rc_ctrl.rc.ch[4]>30)
					{
							pc_shoot.friction_left.speed_set=RAMP_float(fire_friction_30MS_SET,pc_shoot.friction_left.speed_set,5.0f);
							pc_shoot.friction_right.speed_set=RAMP_float(fire_friction_30MS_SET,pc_shoot.friction_right.speed_set,5.0f);		
							PID_Calc(&shoot_pid.left_friction_speed_pid,friction_left_data.speed_rpm,pc_shoot.friction_left.speed_set);
							PID_Calc(&shoot_pid.right_friction_speed_pid,friction_right_data.speed_rpm,-pc_shoot.friction_right.speed_set);
					}
					else if(rc_ctrl.rc.ch[4]<10)
					{
						pc_shoot.friction_left.speed_set=0;
						pc_shoot.friction_right.speed_set=0;
						PID_Calc(&shoot_pid.left_friction_speed_pid,friction_left_data.speed_rpm,0);
						PID_Calc(&shoot_pid.right_friction_speed_pid,friction_right_data.speed_rpm,0);	
						PID_Calc(&shoot_pid.m2006_speed_pid,m2006_data.speed_rpm,0);		
					}
					if(rc_ctrl.rc.ch[4]>450)
					{
						PID_Calc(&shoot_pid.m2006_speed_pid,m2006_data.speed_rpm,fire_booster_30_set);		
					}
					else 
					{
						PID_Calc(&shoot_pid.m2006_speed_pid,m2006_data.speed_rpm,0);					
					}
						pc_shoot.booster_t.current=shoot_pid.m2006_speed_pid.out;
						pc_shoot.friction_left.current=shoot_pid.left_friction_speed_pid.out;
						pc_shoot.friction_right.current=shoot_pid.right_friction_speed_pid.out;
						CAN2_SetMsg_shoot(pc_shoot.booster_t.current,pc_shoot.friction_right.current,pc_shoot.friction_left.current);	
				}					
		}
/***************************************************键盘模式******************************************************/				
		else if(input_mode==pc_mode)
		{
				if(IF_KEY_PRESSED_E&&!E_Flag&&rc_ctrl.key.v)
				{
						E_Flag = 1;
				}				
				if(E_Flag==1&&!IF_KEY_PRESSED_E)
				{
						EE_Flag++;
						EE_Flag%=2;
						E_Flag=0;
				}
				if(IF_KEY_PRESSED_Q&&!Q_Flag&&rc_ctrl.key.v)
				{
						Q_Flag = 1;
				}				
				if(Q_Flag==1&&!IF_KEY_PRESSED_Q)
				{
						QQ_Flag++;
						QQ_Flag%=2;
						Q_Flag=0;
				}					
				if(EE_Flag==1)
			  {
					if( ((28.0f-average_shoot_speed)>=1.0f || (28.0f-average_shoot_speed)>=-1.0f)&&last_shoot_speed!=ShootData.initial_speed&&
						shoot_speed_record[0]>0.0f&&shoot_speed_record[1]>0.0f&&shoot_speed_record[2]>0.0f &&shoot_speed_record[3]>0.0f &&shoot_speed_record[4]>0.0f  )
					{
						fire_friction_30MS_SET+=(28.0f-average_shoot_speed)*1.0f;
					}
					fire_friction_30MS_SET=fp_Limit_mouse(fire_friction_30MS_SET,6500.0f,7800.0f);
					pc_shoot.friction_left.speed_set=fire_friction_30MS_SET;
					pc_shoot.friction_right.speed_set=fire_friction_30MS_SET;
					PID_Calc(&shoot_pid.left_friction_speed_pid,friction_left_data.speed_rpm,pc_shoot.friction_left.speed_set);
					PID_Calc(&shoot_pid.right_friction_speed_pid,friction_right_data.speed_rpm,-pc_shoot.friction_right.speed_set);			
				}
				else if(EE_Flag==0)
				{
					pc_shoot.friction_left.speed_set=0;
					pc_shoot.friction_right.speed_set=0;	
					PID_Calc(&shoot_pid.left_friction_speed_pid,friction_left_data.speed_rpm,pc_shoot.friction_left.speed_set);
					PID_Calc(&shoot_pid.right_friction_speed_pid,friction_right_data.speed_rpm,pc_shoot.friction_right.speed_set);					
				}
				last_shoot_speed=ShootData.initial_speed;					
				if(BB_Flag==0)
				{
					if(rc_ctrl.mouse.press_l==1&& EE_Flag==1)
					{
						left_mouse_last_flag=left_mouse_flag;
						left_mouse_flag=1;
						left_mouse_time++;
						if( (left_mouse_time<200&& left_mouse_flag==1 && left_mouse_last_flag==0))
						{
								pc_shoot.booster_t.angle_set=pc_shoot.booster_t.now_angle+39.5f;
								pc_shoot.shoot_mode=shoot_one;		
						}
						else if(left_mouse_time>=200&& left_mouse_flag==1 && left_mouse_last_flag==1)
						{
								pc_shoot.shoot_mode=shoot_fire;
						}						
					}
					else if(rc_ctrl.mouse.press_l==0)
					{
						left_mouse_last_flag=left_mouse_flag;
						left_mouse_flag=0;
						left_mouse_time=0;
						if(pc_shoot.shoot_mode==shoot_fire)
						{
							PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,0);
							pc_shoot.shoot_mode=shoot_stop;		
						}
					}
				}
				else if(BB_Flag==1)                 //视觉开火
				{
					if(PcData.PcDate.shoot_flag==1&& EE_Flag==1&&rc_ctrl.mouse.press_l==0)
					{					
						if(pc_shoot.shoot_mode==shoot_stop||m2006_data.speed_rpm<500)
						{
								pc_shoot.shoot_mode=shoot_one;
								pc_shoot.booster_t.angle_set=pc_shoot.booster_t.now_angle+39.5f+39.0f;
						}
					}
					else if(EE_Flag==1&&rc_ctrl.mouse.press_l==1)
					{				
								pc_shoot.shoot_mode=shoot_fire;										
					}
					else
					{
						if(pc_shoot.shoot_mode==shoot_fire)
						{
							PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,0);
							pc_shoot.shoot_mode=shoot_stop;		
						}
					}					
				}
				if(pc_shoot.shoot_mode==shoot_one)
				{
					PID_Calc(&shoot_pid.m2006_ecd_pid,pc_shoot.booster_t.now_angle,pc_shoot.booster_t.angle_set);
					PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,shoot_pid.m2006_ecd_pid.out);					
					shoot_error=wheel_fabsf(pc_shoot.booster_t.angle_set-pc_shoot.booster_t.now_angle);	
//					if(shoot_error<0.5f)
//					{				
//						pc_shoot.shoot_mode=shoot_stop;	
//					}				
				}
				else if(pc_shoot.shoot_mode==shoot_fire)                        //热量限制
				{
					switch (GameRobotStat.shooter_barrel_heat_limit)
					{
						case 200:
						heat_limit=20.0f;
						if(pc_shoot.rest_heat<=30.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3000+QQ_Flag*2500.0f;              //5500是拨盘能漏弹的极限，速度再快子弹无法掉入洞中
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*13.0f;
						}
						break;							
						case 250:
						heat_limit=20.0f;
						if(pc_shoot.rest_heat<=20.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3200+QQ_Flag*2300.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*13.0f;
						}
						break;								
						case 300:	
							heat_limit=20.0f;
						if(pc_shoot.rest_heat<=31.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3400+QQ_Flag*1900.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*13.0f;		
						}
						break;							
						case 350:
							heat_limit=25.0f;
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3200+QQ_Flag*1900.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*10.0f;
						}
						break;							
						case 400:
						heat_limit=20.0f;			
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3200+QQ_Flag*2100.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*10.0f;
						}
						break;						
						case 450:
						heat_limit=30.0f;			
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3200+QQ_Flag*1600.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*10.0f;				
						}							
						break;							
						case 500:
						heat_limit=20.0f;			
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>51.0f)
						{
							pc_shoot.booster_t.speed_set=3400+QQ_Flag*1800.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*10.0f;				
						
						}							
						break;
						case 550:
						heat_limit=30.0f;		
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3400+QQ_Flag*1600.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*8.0f;			
						
						}	
						break;						
						case 600:
						heat_limit=30.0f;
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>61.0f)
						{
							pc_shoot.booster_t.speed_set=3500+QQ_Flag*1700.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*7.0f;				
						
						}
						break;							
						case 650:
							heat_limit=20.0f;
						if(pc_shoot.rest_heat<=51.0f)
						{
							pc_shoot.booster_t.speed_set=500;
						}							
						else if(pc_shoot.rest_heat>51.0f)
						{
							pc_shoot.booster_t.speed_set=3200+QQ_Flag*2100.0f;
						}
						else
						{
							pc_shoot.booster_t.speed_set=500+pc_shoot.rest_heat*10.0f;				
						
						}
						break;
						default:					
						heat_limit=20.0f;						
						pc_shoot.booster_t.speed_set=3200;	
					}				
					PID_Calc(&shoot_pid.m2006_speed_pid,m2006_data.speed_rpm,pc_shoot.booster_t.speed_set);
				}	
				else if(pc_shoot.shoot_mode==shoot_stop)
				{
					PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,0);
				}
				else if(pc_shoot.shoot_mode==shoot_return)
				{
					PID_Calc(&shoot_pid.m2006_ecd_pid,pc_shoot.booster_t.now_angle,pc_shoot.booster_t.angle_set);
					PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,shoot_pid.m2006_ecd_pid.out);					
					shoot_error=wheel_fabsf(pc_shoot.booster_t.angle_set-pc_shoot.booster_t.now_angle);
//					if(shoot_error<0.5f)
//					{				
//						pc_shoot.shoot_mode=shoot_stop;	
//					}							
				}
				else
				{
					pc_shoot.shoot_mode=shoot_stop;	
				}
				if(EE_Flag==1&&(m2006_data.given_current>8000||m2006_data.given_current<-8000)&&rc_ctrl.mouse.press_l==1 )
				{
					pc_shoot.booster_t.angle_set=pc_shoot.booster_t.now_angle-40.0f;
					pc_shoot.shoot_mode=shoot_return;					
				}				
				pc_shoot.booster_t.current=shoot_pid.m2006_speed_pid.out;
				pc_shoot.friction_left.current=shoot_pid.left_friction_speed_pid.out;
				pc_shoot.friction_right.current=shoot_pid.right_friction_speed_pid.out;			
				if(pc_shoot.rest_heat<heat_limit)
				{
					PID_Calc(&shoot_pid.m2006_speed_pid,pc_shoot.booster_t.now_speed,0);
					pc_shoot.booster_t.current=shoot_pid.m2006_speed_pid.out;					
				}
				CAN2_SetMsg_shoot(pc_shoot.booster_t.current,pc_shoot.friction_right.current,pc_shoot.friction_left.current);				
		}
}





