#include "chassis_task.h"
#include "can.h"

extern DM_motor_measure_t DM_data[4];
extern M1505B_typedef M1505B_data[2];
extern gimbal_typedef yaw_motor;
extern int input_mode;
extern int last_input_mode;
extern float Z_M;

float L5=0.210f;   //AE
float L1=0.144f;   
float L2=0.2755f;
float L3=0.2755f;
float L4=0.144f;

float test_flag=0;
/*滑动平均滤波器*////////
int silide_N=50;
float L_silide_buff[100];
float L_silide_sum=0;

float R_silide_buff[100];
float R_silide_sum=0;

float recover_flag=0;
vmc_leg leg_left,leg_right;

uint64_t Time_L_Last;																								//全局定时器数值变量，用于记录上一次运行到此处时的时间，与下一次时间做差求运行时间
uint64_t Time_L_Delta;
uint64_t Time_L_RUN;

uint64_t Time_R_Last;																								//全局定时器数值变量，用于记录上一次运行到此处时的时间，与下一次时间做差求运行时间
uint64_t Time_R_Delta;
uint64_t Time_R_RUN;

float err_speed1,err_speed2;


//////////////////////////////////////////////////////////////////*********************************LQR参数*****************************************///////////////////////////////////////////////////////////////

float Poly_Coefficient[12][4]=
{
	
	
	
	
	//此部分经由Matlab求解得到，通过excel直接复制到代码内
//	
//    Q=diag([10 1 500 100 6000 1]);
//    R=[1 0;0 0.25];    
	
//{-205.7756005,	277.8417651,	-234.6509652,	-0.014665936},
//{21.50197206	,-27.13360282,	-24.22416723,	0.252587927},
//{-288.3736758,	288.1329677,	-100.1857016,	-9.425655915},
//{-176.4540779,	184.4773689,	-77.6671394	,-7.578299246},
//{-704.8499938,	846.7262086,	-389.8849571,	84.50326753},
//{-24.42677862,	35.58596934,	-20.51334433,	6.438921561},
//{345.692552,	-299.2942967,	67.68398091,	20.40472759},
//{42.03368475,	-43.07245095	,16.53885397	,2.212699045},
//{-406.9435676	,487.7338854	,-223.5918046,	47.51998583},
//{-310.9035302,	361.6128796,	-161.5811072,	35.36468737},
//{2076.526069,	-2076.459794	,723.5512175,	61.71426138},
//{132.018923,	-138.0032256,	51.62089593,	-0.15872029}

{-244.684325,	317.5737313	,-249.1924378,	-0.350373469},
{19.2460215,	-24.67263321,	-25.4581015,	0.274049208},
{-285.3691677,	283.6828141,	-97.93412708,	-9.846693881},
{-178.3747258,	185.1857788,	-77.27694467,	-7.818874245},
{-745.7067938,	880.5581088	,-397.3557779,	84.15282164},
{-28.34656519	,38.96647443,	-21.23783932,	6.292847089},
{341.9513271,	-292.0097512,	63.1000767,	21.36703785},
{42.56926244,	-43.26359514,	16.32285991,	2.265602428},
{-429.8539626,	506.3616187	,-227.4296662,	47.18870163},
{-324.0498684	,372.7937052,	-164.2024979,	35.24112292},
{2060.369321,	-2050.180572,	709.4851152	,64.61289058},
{128.2305765,	-132.9869895,	49.18820284,	0.32182514}






	

};
float wheel_fabsf(float input)
{
	if(input>=0) return input;
	else return -input;
}

int16_t rc_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }

    return Value;
}

float fp_deadline(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }

    return Value;
}


/*********************************************************************超级电容**************************************************************************************/
void Super_power_ctrl()
{
			
//    if(GameState.game_progress== 4)   //比赛中
//    {
//      	CAN_CMD_SUPERPOWER(GameRobotStat.chassis_power_limit,1,PowerHeatData.chassis_power_buffer);//切换电源
//		}    
//		else if(chassis_monitor_point.errorExist == 0)//裁判系统离线，使用电源功率
//    {
				CAN_CMD_SUPERPOWER(0,2,0);
//    }
	
}

void chassis_L_init()//1200 30000   300
{
	leg_left.collect_leg_pid.Kp=1000.0f; leg_left.collect_leg_pid.Ki=0.0f; leg_left.collect_leg_pid.Kd=5000.0f; leg_left.collect_leg_pid.max_iout=0;    leg_left.collect_leg_pid.max_out=500.0f;
	leg_left.leg_pid.Kp=1000;	  leg_left.leg_pid.Ki=0;    leg_left.leg_pid.Kd=20000;   leg_left.leg_pid.max_iout=0;    leg_left.leg_pid.max_out=400.0f;
	leg_left.turn_pid.Kp=0.4f; 	leg_left.turn_pid.Ki=0; 	leg_left.turn_pid.Kd=0.1f; 	leg_left.turn_pid.max_iout=0; 	leg_left.turn_pid.max_out=10;
	leg_left.theta_error_pid.Kp=40; 	leg_left.theta_error_pid.Ki=0; 	leg_left.theta_error_pid.Kd=0.0f; 	leg_left.theta_error_pid.max_out=10; 	leg_left.theta_error_pid.max_iout=0;
	leg_left.roll_pid.Kp=2.0f;	  leg_left.roll_pid.Ki=0.0f;    leg_left.roll_pid.Kd=-0.3f;   leg_left.roll_pid.max_iout=0;    leg_left.roll_pid.max_out=60.0f;//40
	
	pid_init(&leg_left.leg_pid,PID_POSITION,leg_left.leg_pid.Kp,leg_left.leg_pid.Ki,leg_left.leg_pid.Kd,leg_left.leg_pid.max_out,leg_left.leg_pid.max_iout);	
	pid_init(&leg_left.turn_pid,PID_POSITION,leg_left.turn_pid.Kp,leg_left.turn_pid.Ki,leg_left.turn_pid.Kd,leg_left.turn_pid.max_out,leg_left.turn_pid.max_iout);		
	pid_init(&leg_left.theta_error_pid,PID_POSITION,leg_left.theta_error_pid.Kp,leg_left.theta_error_pid.Ki,leg_left.theta_error_pid.Kd,leg_left.theta_error_pid.max_out,leg_left.theta_error_pid.max_iout);			
	pid_init(&leg_left.roll_pid,PID_POSITION,leg_left.roll_pid.Kp,leg_left.roll_pid.Ki,leg_left.roll_pid.Kd,leg_left.roll_pid.max_out,leg_left.roll_pid.max_iout);		
	
}

void chassis_R_init()
{	
	leg_right.collect_leg_pid.Kp=1000.0f; leg_right.collect_leg_pid.Ki=0.0f; leg_right.collect_leg_pid.Kd=5000.0f; leg_right.collect_leg_pid.max_iout=0;    leg_right.collect_leg_pid.max_out=500.0f;	
	leg_right.right_leg_pid.Kp=1000;	  leg_right.right_leg_pid.Ki=0;    leg_right.right_leg_pid.Kd=20000;   leg_right.right_leg_pid.max_iout=0;    leg_right.right_leg_pid.max_out=400.0f;
	
	pid_init(&leg_right.right_leg_pid,PID_POSITION,leg_right.right_leg_pid.Kp,leg_right.right_leg_pid.Ki,leg_right.right_leg_pid.Kd,leg_right.right_leg_pid.max_out,leg_right.right_leg_pid.max_iout);
	
}


void chassis_L_task()
{
		Time_L_RUN=Time_Add;
		Time_L_Delta = Time_L_RUN-Time_L_Last;
		Time_L_Last=Time_L_RUN;
	
		Kalaman_feedback(&Kalman0,(float)Time_L_Delta/10000.0f,M1505B_data[0].speed_rpm*0.01047197f*M1505B_R,MPU9250_Real_Data.Accel_X*arm_cos_f32(Angular_Handler.ROLL*TheattoRad));

		err_speed1=M1505B_data[0].speed_rpm*0.01047197f*M1505B_R- Kalman0.xhat_data[0];				//用于查看在未打滑状态下卡尔曼滤波器的准确度	
		rc_ctrl.rc.ch[1]=rc_deadline(rc_ctrl.rc.ch[1],-RC_deadline,RC_deadline);
		if(input_mode==stop_mode)
		{
			leg_left.step_flag=0;
			leg_left.step_time=0;
			leg_left.recover_flag=0;
			CAN1_SetMsg_damiao_exit(0x12);		
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_exit(0x13);						
//			Can_9025_Send1(0x80,0);
		}
		else if(input_mode==init_mode)
		{
			leg_left.last_yaw=Angular_Handler.YAW;			
			CAN1_SetMsg_damiao_start(0x12);		
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_start(0x13);	
			vTaskDelay(1);
//			Can_M1505b_ID(2);	
		}
//		else if(input_mode==zero_mode)
//		{
//			CAN1_SetMsg_damiao_zero(0x12);		
//			vTaskDelay(1);			
//			CAN1_SetMsg_damiao_zero(0x13);						
//		}
		leg_left.fai1=3.1415926f+DM_data[2].position+0.349f;
		leg_left.fai4=DM_data[3].position-0.349f;
		
	/*****************left************************/	
		leg_left.YD=L4*arm_sin_f32(leg_left.fai4);
		leg_left.YB=L1*arm_sin_f32(leg_left.fai1);
		leg_left.XD=L5+L4*arm_cos_f32(leg_left.fai4);
		leg_left.XB=L1*arm_cos_f32(leg_left.fai1);	
		
		leg_left.LBD=sqrt((leg_left.XD-leg_left.XB)*(leg_left.XD-leg_left.XB)+(leg_left.YD-leg_left.YB)*(leg_left.YD-leg_left.YB));
		leg_left.A0 = 2.0f*L2*(leg_left.XD - leg_left.XB);
		leg_left.B0 = 2.0f*L2*(leg_left.YD - leg_left.YB);
		leg_left.C0 = L2*L2 + leg_left.LBD*leg_left.LBD - L3*L3;	
		leg_left.fai2=2.0f*atan2f((leg_left.B0+sqrt(leg_left.A0*leg_left.A0+leg_left.B0*leg_left.B0-leg_left.C0*leg_left.C0)),leg_left.A0+leg_left.C0);
		leg_left.fai3=atan2f(leg_left.YB-leg_left.YD+L2*arm_sin_f32(leg_left.fai2),leg_left.XB-leg_left.XD+L2*arm_cos_f32(leg_left.fai2));
		
		leg_left.XC=L1*arm_cos_f32(leg_left.fai1)+L2*arm_cos_f32(leg_left.fai2);
		leg_left.YC=L1*arm_sin_f32(leg_left.fai1)+L2*arm_sin_f32(leg_left.fai2);
		
		leg_left.last_L0=leg_left.L0;
		leg_left.L0=sqrt((leg_left.XC-L5/2.0f)*(leg_left.XC-L5/2.0f)+leg_left.YC*leg_left.YC);    //L0
		leg_left.last_d_L0=leg_left.d_L0;
		leg_left.d_L0=(leg_left.L0-leg_left.last_L0)/Chassis_task_Time;                //d_L0
		leg_left.d2_L0=(leg_left.d_L0-leg_left.last_d_L0)/Chassis_task_Time;      
		
		leg_left.fai0=atan2f(leg_left.YC,(leg_left.XC-L5/2.0f));
		leg_left.last_theta=leg_left.theta;
		leg_left.theta=3.1415926f/2.0f-(Angular_Handler.ROLL-MPU_offect)*TheattoRad-leg_left.fai0;     //theta

		leg_left.last_d_theta=leg_left.d_theta;
		leg_left.d_theta=(leg_left.theta-leg_left.last_theta)/Chassis_task_Time;   //d_theta
		if(leg_left.d_theta>20||leg_left.d_theta<-20)
		{
			leg_left.d_theta=leg_left.last_d_theta;
		}
			for(int i=0;i<silide_N-1;i++)
			{
				L_silide_buff[i+1]=L_silide_buff[i];
			}
			L_silide_buff[0]=leg_left.d_theta;
			for(int i=0;i<silide_N;i++)
			{
				L_silide_sum+=L_silide_buff[i];
			}
			leg_left.d_theta_filter=L_silide_sum/silide_N;			
			L_silide_sum=0;
		leg_left.d2_theta=(leg_left.d_theta-leg_left.last_d_theta)/Chassis_task_Time;
		
		if(last_input_mode==remote_mode&&input_mode==spin_mode)
		{
			leg_left.step_flag=1;	
		}
		else if(input_mode==remote_mode)
		{
			leg_left.step_flag=0;	
		}		
		leg_left.theta_error_T=PID_Calc(&leg_left.theta_error_pid,leg_right.theta-leg_left.theta,0);
		
		for(int j=0;j<12;j++)
		{
			leg_left.LQR_K[j]=Poly_Coefficient[j][0]*leg_left.L0*leg_left.L0*leg_left.L0+Poly_Coefficient[j][1]*leg_left.L0*leg_left.L0+Poly_Coefficient[j][2]*leg_left.L0+Poly_Coefficient[j][3];
		}
				
		if(leg_left.recover_flag==1&&leg_right.leave_leg_flag==1&&leg_left.leave_leg_flag==1&&leg_left.v_set<0.005f)
		{
			leg_left.x_filter=leg_left.x_filter-Kalman0.xhat_data[0]*Chassis_task_Time;
		}
		else
		{
			leg_left.x_filter=0;
		}
		
		leg_left.v_error=-Kalman0.xhat_data[0]-leg_left.v_set;
		leg_left.v_error=fp_Limit_mouse(leg_left.v_error,-2.5f,2.5f);
		
		leg_left.wheel_T=leg_left.LQR_K[0]*(leg_left.theta-0.0f)        //负为正向
										+leg_left.LQR_K[1]*(leg_left.d_theta_filter*wheel_theta_K-0.0f)
//										 +leg_left.LQR_K[2]*(leg_left.x_filter-0)*0.1f
										 +leg_left.LQR_K[3]*(leg_left.v_error)*0.4f                 //-
										 +leg_left.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
									   +leg_left.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;
		
				 leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)      //+
										 -leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f)        //-
//										 -leg_left.LQR_K[8]*(leg_left.x_filter-0)*0.1f
  									 -leg_left.LQR_K[9]*(leg_left.v_error)*0.4f               //+
										 -leg_left.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad         //-
										 -leg_left.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;               //-
		leg_left.Tp=leg_left.Tp-leg_left.theta_error_T;
		
		leg_left.L0_set=0.20f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f-0.20f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);	
		leg_left.L0_set=fp_Limit_mouse(leg_left.L0_set,Min_leg,Max_leg);

		leg_left.ROLL_T=Roll_PID_Calc(&leg_left.roll_pid,0,Angular_Handler.Pitch+pitch_offect,Angular_Handler.V_Y,1.0f);

		if(leg_left.step_flag==0&&test_flag==0)
		{
			leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set)+leg_left.ROLL_T; 		  //L0 (0.15~0.4)  
		}
		else if(leg_left.step_flag==1&&test_flag==0&&(Angular_Handler.ROLL-MPU_offect)>-8.0f&&leg_left.step_time==0)
		{
			leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,step_leg_set);
		}
		else if( ( leg_left.recover_flag==1 && (Angular_Handler.ROLL-MPU_offect)<-8.0f && leg_left.step_flag==1 )|| test_flag==1||leg_left.step_time>0)
		{
			leg_left.F0=100.0f-PID_Calc(&leg_left.collect_leg_pid,leg_left.L0,0.1f);
			if(leg_left.L0<0.15f)
			{
				leg_left.step_time++;
			}
		  if(leg_left.step_time>=100&&leg_right.step_time>=100)
		  { 
				 leg_left.step_time=0;
				 leg_right.step_time=0;
				 leg_left.step_flag=0;				
				 leg_right.step_flag=0;
		  }			
		}
			
		leg_left.j11= (L1*arm_sin_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j12= (L1*arm_cos_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		leg_left.j21= (L4*arm_sin_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j22= (L4*arm_cos_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		
		leg_left.leg_T[0]=leg_left.j11*leg_left.F0+leg_left.j12*leg_left.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_left.leg_T[1]=leg_left.j21*leg_left.F0+leg_left.j22*leg_left.Tp;//Tp为沿中心轴的力矩 

		leg_left.v_set=rc_ctrl.rc.ch[1]/660.0f*2.0f;
		leg_right.v_set=leg_left.v_set;
	
		leg_left.turn_T=turn_PID_Calc(&leg_left.turn_pid,Angular_Handler.YAW,leg_left.yaw_set,Angular_Handler.V_Z);
		leg_left.wheel_T=leg_left.wheel_T+leg_left.turn_T;
/****************离地检测*****************/			
		leg_left.un_F0=(-arm_cos_f32(leg_left.fai0-leg_left.fai2)*DM_data[2].torque+arm_cos_f32(leg_left.fai0-leg_left.fai3)*DM_data[3].torque)/(leg_left.L0*L1*arm_sin_f32(leg_left.fai1-leg_left.fai2));
		leg_left.un_Tp=(arm_sin_f32(leg_left.fai0-leg_left.fai2)*DM_data[2].torque-arm_sin_f32(leg_left.fai0-leg_left.fai3)*DM_data[3].torque)/(arm_sin_f32(leg_left.fai3-leg_left.fai2)*L1*arm_sin_f32(leg_left.fai1-leg_left.fai2));
		
		leg_left.Z_W=Z_M-leg_left.d2_L0*arm_cos_f32(leg_left.theta)+2.0f*leg_left.d_L0*leg_left.d_theta*arm_sin_f32(leg_left.theta)
		+leg_left.L0*leg_left.d2_theta*arm_sin_f32(leg_left.theta)+leg_left.L0*leg_left.d_theta*leg_left.d_theta*arm_cos_f32(leg_left.theta);
		
		leg_left.F1=leg_left.un_F0*arm_cos_f32(leg_left.theta);
		leg_left.F2=leg_left.un_Tp*arm_sin_f32(leg_left.theta)/leg_left.L0;
		leg_left.FN[3]=leg_left.un_F0*arm_cos_f32(leg_left.theta)-leg_left.un_Tp*arm_sin_f32(leg_left.theta)/leg_left.L0+wheel_mg+leg_left.Z_W;   //支持力		
		leg_left.FN[0]=leg_left.FN[1];
		leg_left.FN[1]=leg_left.FN[2];
		leg_left.FN[2]=leg_left.FN[3];
		leg_left.FN_filter=0.25f*leg_left.FN[0]+0.25f*leg_left.FN[1]+0.25f*leg_left.FN[2]+0.25f*leg_left.FN[3];		
		if(leg_left.recover_flag==1&&leg_left.FN_filter<60)
		{
				leg_left.leave_leg_flag=1;
		}
		else
		{
				leg_left.leave_leg_flag=0;
		}
		if(leg_left.recover_flag==1&&leg_right.leave_leg_flag==1&&leg_left.leave_leg_flag==1)
		{
				leg_left.v_set=0;
				leg_left.wheel_T=0;
				leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)-leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f);				//-
		}		
/****************离地检测*****************/	
		leg_left.leg_T[0]=-fp_Limit_mouse(leg_left.leg_T[0],-30.0f,30.0f);
		leg_left.leg_T[1]=-fp_Limit_mouse(leg_left.leg_T[1],-30.0f,30.0f);		
		leg_left.wheel_T=fp_Limit_mouse(leg_left.wheel_T*744.7f,-16384.0f,16384.0f);
		if(input_mode==remote_mode||input_mode==spin_mode)
		{
			if(last_input_mode==init_mode)
			{
				leg_left.yaw_set=leg_left.last_yaw;
			}
			leg_left.yaw_set+=rc_ctrl.rc.ch[0]/660.0f/10.0f*3.0f;
			Can_M1505b_I(leg_left.wheel_T,leg_right.wheel_T);			
			vTaskDelay(1);
			if(fabs(Angular_Handler.ROLL-MPU_offect)<3.0f)
			{
				leg_left.recover_flag=1;
			}
//			else if(fabs(Angular_Handler.ROLL-MPU_offect)>20.0f)
//			{
//				leg_left.recover_flag=0;
//			}			
			if(leg_left.recover_flag==1)
			{
				DM_MIT(0x12,leg_left.leg_T[0]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x13,leg_left.leg_T[1]);			
			}
			else if(leg_left.recover_flag==0)
			{			
//				DM_MIT(0x12,0);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
//				DM_MIT(0x13,0);				
			}
		}

}


void chassis_R_task()
{
		Time_R_RUN=Time_Add;
		Time_R_Delta = Time_R_RUN-Time_R_Last;
		Time_R_Last=Time_R_RUN;	

		Kalaman_feedback(&Kalman2,(float)Time_R_Delta/10000.0f,M1505B_data[1].speed_rpm*0.01047197f*M1505B_R,MPU9250_Real_Data.Accel_X*arm_cos_f32(Angular_Handler.ROLL*TheattoRad));

		err_speed2=M1505B_data[1].speed_rpm*0.01047197f*M1505B_R- Kalman2.xhat_data[0];
	
		if(input_mode==stop_mode)
		{			
			leg_right.recover_flag=0;
			leg_right.step_flag=0;
			leg_right.step_time=0;			
			CAN1_SetMsg_damiao_exit(0x10);
			vTaskDelay(1);
			CAN1_SetMsg_damiao_exit(0x11);		
			vTaskDelay(1);		
			Can_M1505b_I(0,0);
		}
		else if(input_mode==init_mode)
		{		
			CAN1_SetMsg_damiao_start(0x10);	
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_start(0x11);				
//			Can_9025_Send2(0x88,0);			
		}	
//		else if(input_mode==zero_mode)
//		{
//			CAN1_SetMsg_damiao_zero(0x10);		
//			vTaskDelay(1);			
//			CAN1_SetMsg_damiao_zero(0x11);							
//		}		
		leg_right.fai1 = 3.1415926f-(DM_data[0].position+0.0f)+0.349f;
		leg_right.fai4 = -(DM_data[1].position-0.0f)-0.349f;//3jiadou180	

	/*****************right************************/	
		leg_right.YD=L4*arm_sin_f32(leg_right.fai4);
		leg_right.YB=L1*arm_sin_f32(leg_right.fai1);
		leg_right.XD=L5+L4*arm_cos_f32(leg_right.fai4);
		leg_right.XB=L1*arm_cos_f32(leg_right.fai1);	
		
		leg_right.LBD=sqrt((leg_right.XD-leg_right.XB)*(leg_right.XD-leg_right.XB)+(leg_right.YD-leg_right.YB)*(leg_right.YD-leg_right.YB));
		leg_right.A0 = 2.0f*L2*(leg_right.XD - leg_right.XB);
		leg_right.B0 = 2.0f*L2*(leg_right.YD - leg_right.YB);
		leg_right.C0 = L2*L2 + leg_right.LBD*leg_right.LBD - L3*L3;	
		leg_right.fai2=2.0f*atan2f((leg_right.B0+sqrt(leg_right.A0*leg_right.A0+leg_right.B0*leg_right.B0-leg_right.C0*leg_right.C0)),leg_right.A0+leg_right.C0);
		leg_right.fai3=atan2f(leg_right.YB-leg_right.YD+L2*arm_sin_f32(leg_right.fai2),leg_right.XB-leg_right.XD+L2*arm_cos_f32(leg_right.fai2));
		
		leg_right.XC=L1*arm_cos_f32(leg_right.fai1)+L2*arm_cos_f32(leg_right.fai2);
		leg_right.YC=L1*arm_sin_f32(leg_right.fai1)+L2*arm_sin_f32(leg_right.fai2);
		
		leg_right.last_L0=leg_right.L0;
		leg_right.L0=sqrt((leg_right.XC-L5/2.0f)*(leg_right.XC-L5/2.0f)+leg_right.YC*leg_right.YC);    //L0
		leg_right.last_d_L0=leg_right.d_L0;
		leg_right.d_L0=(leg_right.L0-leg_right.last_L0)/Chassis_task_Time;
		leg_right.d2_L0=(leg_right.d_L0-leg_right.last_d_L0)/0.003f;
		
		leg_right.fai0=atan2f(leg_right.YC,(leg_right.XC-L5/2.0f));
		leg_right.last_theta=leg_right.theta;
		leg_right.theta=3.1415926f/2.0f-(Angular_Handler.ROLL-MPU_offect)*TheattoRad-leg_right.fai0;     //theta
	/////******* 滑动平均滤波器********////	
			for(int i=0;i<silide_N-1;i++)
			{
				R_silide_buff[i+1]=R_silide_buff[i];
			}
			R_silide_buff[0]=leg_right.d_theta;
			for(int i=0;i<silide_N;i++)
			{
				R_silide_sum+=R_silide_buff[i];
			}
			leg_right.d_theta_filter=R_silide_sum/silide_N;			
			R_silide_sum=0;		
	/*****************///////////	
		leg_right.last_d_theta=leg_right.d_theta;
		leg_right.d_theta=(leg_right.theta-leg_right.last_theta)/Chassis_task_Time;                 //d_theta
		if(leg_right.d_theta>20||leg_right.d_theta<-20)
		{
			leg_right.d_theta=leg_right.last_d_theta;
		}
		leg_right.d2_theta=(leg_right.d_theta-leg_right.last_d_theta)/Chassis_task_Time;	

		if(last_input_mode==remote_mode&&input_mode==spin_mode)
		{
			leg_right.step_flag=1;	
		}
		else if(input_mode==remote_mode)
		{
			leg_right.step_flag=0;	
		}		
		for(int j=0;j<12;j++)
		{
			leg_right.LQR_K[j]=Poly_Coefficient[j][0]*leg_right.L0*leg_right.L0*leg_right.L0+Poly_Coefficient[j][1]*leg_right.L0*leg_right.L0+Poly_Coefficient[j][2]*leg_right.L0+Poly_Coefficient[j][3];
		}
	//9025电机直径89mm	
//		leg_right.x_set=leg_right.x_set+leg_right.v_set*Chassis_task_Time;
		if(leg_right.recover_flag==1&&leg_right.leave_leg_flag==1&&leg_left.leave_leg_flag==1&&leg_right.v_set<0.005f)
		{
			leg_right.x_filter=leg_right.x_filter+Kalman2.xhat_data[0]*Chassis_task_Time;
		}
		else
		{
			leg_right.x_filter=0;
		}
		
		leg_right.v_error=Kalman2.xhat_data[0]-leg_right.v_set;
		leg_left.v_error=fp_Limit_mouse(leg_left.v_error,-2.5f,2.5f);
		
		leg_right.wheel_T=leg_right.LQR_K[0]*(leg_right.theta-0.0f)
										  +leg_right.LQR_K[1]*(leg_right.d_theta_filter*wheel_theta_K-0.0f)
//											+leg_right.LQR_K[2]*(leg_right.x_filter-0)*0.1f
										  +leg_right.LQR_K[3]*(leg_right.v_error)*0.4f                   //+   
										  +leg_right.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										  +leg_right.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;
		
				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)         			 //-
										 -leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f)           //-
//										 -leg_right.LQR_K[8]*(leg_right.x_filter-0)*0.1f
										 -leg_right.LQR_K[9]*(leg_right.v_error)*0.4f                //+
										 -leg_right.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad             //+
										 -leg_right.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;	                   //-
		
		leg_right.Tp=leg_right.Tp+leg_left.theta_error_T;		
		
		leg_right.L0_set=0.20f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f+0.20f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);
		leg_right.L0_set=fp_Limit_mouse(leg_right.L0_set,Min_leg,Max_leg);

		if(leg_right.step_flag==0&&test_flag==0)
		{
			leg_right.F0= -all_Mg/arm_cos_f32(leg_right.theta)-PID_Calc(&leg_right.right_leg_pid,leg_right.L0,leg_right.L0_set)-leg_right.ROLL_T; 		  //L0 (0.15~0.4)  
		}
		else if(leg_right.step_flag==1&&test_flag==0&&(Angular_Handler.ROLL-MPU_offect)>-8.0f&&leg_right.step_time==0)
		{
			leg_right.F0= -all_Mg/arm_cos_f32(leg_right.theta)-PID_Calc(&leg_right.right_leg_pid,leg_right.L0,step_leg_set)-leg_right.ROLL_T;
		}
		else if( (leg_right.recover_flag==1&&Angular_Handler.ROLL-MPU_offect<-8.0f&&leg_right.step_flag==1) ||test_flag==1||leg_right.step_time>0)
		{
			leg_right.F0=100.0f-PID_Calc(&leg_right.collect_leg_pid,leg_right.L0,0.1f);
			if(leg_right.L0<0.15f)
			{
				leg_right.step_time++;
			}			
		  if(leg_right.step_time>=100&&leg_left.step_time>=100)
		  { 
				 leg_left.step_time=0;
				 leg_right.step_time=0;
				 leg_left.step_flag=0;				
				 leg_right.step_flag=0;
				test_flag=0;
		  }			
		}		

		leg_right.j11= (L1*arm_sin_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j12= (L1*arm_cos_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
		leg_right.j21= (L4*arm_sin_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j22= (L4*arm_cos_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));

		leg_right.leg_T[0]=leg_right.j11*leg_right.F0+leg_right.j12*leg_right.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_right.leg_T[1]=leg_right.j21*leg_right.F0+leg_right.j22*leg_right.Tp;//Tp为沿中心轴的力矩 
		leg_right.wheel_T=leg_right.wheel_T-leg_left.turn_T;
/****************离地检测*****************/		
		leg_right.un_F0=(-arm_cos_f32(leg_right.fai0-leg_right.fai2)*DM_data[2].torque+arm_cos_f32(leg_right.fai0-leg_right.fai3)*DM_data[3].torque)/(leg_right.L0*L1*arm_sin_f32(leg_right.fai1-leg_right.fai2));
		leg_right.un_Tp=(arm_sin_f32(leg_right.fai0-leg_right.fai2)*DM_data[2].torque-arm_sin_f32(leg_right.fai0-leg_right.fai3)*DM_data[3].torque)/(arm_sin_f32(leg_right.fai3-leg_right.fai2)*L1*arm_sin_f32(leg_right.fai1-leg_right.fai2));

		leg_right.Z_W=Z_M-leg_right.d2_L0*arm_cos_f32(leg_right.theta)+2.0f*leg_right.d_L0*leg_right.d_theta*arm_sin_f32(leg_right.theta)
		+leg_right.L0*leg_right.d2_theta*arm_sin_f32(leg_right.theta)+leg_left.L0*leg_right.d_theta*leg_right.d_theta*arm_cos_f32(leg_right.theta);
		
		leg_right.FN[3]=leg_right.un_F0*arm_cos_f32(leg_right.theta)-leg_right.un_Tp*arm_sin_f32(leg_right.theta)/leg_right.L0+wheel_mg+leg_right.Z_W;   //支持力
		leg_right.FN[0]=leg_right.FN[1];
		leg_right.FN[1]=leg_right.FN[2];
		leg_right.FN[2]=leg_right.FN[3];
		leg_right.FN_filter=0.25f*leg_right.FN[0]+0.25f*leg_right.FN[1]+0.25f*leg_right.FN[2]+0.25f*leg_right.FN[3];
		if(leg_right.recover_flag==1&&leg_right.FN_filter<10)
		{
				leg_right.leave_leg_flag=1;
		}
		else
		{
				leg_right.leave_leg_flag=0;
		}
		if(leg_right.recover_flag==1&&leg_right.leave_leg_flag==1&&leg_left.leave_leg_flag==1)
		{
				leg_right.v_set=0;
				leg_right.wheel_T=0;
				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)-leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f);				//-	
		}	
/****************离地检测*****************/			
		leg_right.leg_T[0]=fp_Limit_mouse(leg_right.leg_T[0],-30.0f,30.0f);
		leg_right.leg_T[1]=fp_Limit_mouse(leg_right.leg_T[1],-30.0f,30.0f);		
		leg_right.wheel_T=-fp_Limit_mouse(leg_right.wheel_T*744.7f,-16384.0f,16384.0f);
		if(input_mode==remote_mode||input_mode==spin_mode)
		{		
			Can_M1505b_I(leg_left.wheel_T,leg_right.wheel_T);
			vTaskDelay(1);
			if(fabs(Angular_Handler.ROLL-MPU_offect)<3.0f)
			{
				leg_right.recover_flag=1;
			}
//			else if(fabs(Angular_Handler.ROLL-MPU_offect)>20.0f)
//			{
//				leg_right.recover_flag=0;
//			}
			if(leg_right.recover_flag==1)
			{
				DM_MIT(0x10,leg_right.leg_T[0]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x11,leg_right.leg_T[1]);			
			}
			else if(leg_right.recover_flag==0)
			{
//				DM_MIT(0x10,0);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
//				DM_MIT(0x11,0);
			}
		}

}














