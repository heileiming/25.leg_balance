#include "chassis_task.h"
#include "can.h"

extern DM_motor_measure_t DM_data[4];
extern M1505B_typedef M1505B_data[2];
extern gimbal_typedef yaw_motor;
extern int input_mode;
extern int last_input_mode;

float L5=0.210f;   //AE
float L1=0.144f;   
float L2=0.2755f;
float L3=0.2755f;
float L4=0.144f;

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
//    Q=diag([150 50 0.0001 10 5000 300]);
//    R=[1 0;0 0.25]; 
	
//{-371.9268175,	412.7962771	,-189.7959387	,-8.254977463},
//{-107.6743614,	106.3191318	,-45.11962055	,-4.050413459},
//{-0.035833836,	0.038611673	,-0.014302591	,-0.008147306},
//{-22.53444947,	24.29125191	,-9.000147215	,-5.167460804},
//{-591.1549605,	778.6637939	,-390.0443303	,85.07890378},
//{-136.3565223,	179.6665927	,-89.19019853	,19.50700489},
//{126.4769031,	14.51826953	,-92.22117198	,40.59755281},
//{-130.2323957,	181.2269965,	-96.05387175,	20.95629266},
//{-0.047883122,	0.098811261,	-0.063464754,	0.014086586},
//{-30.6712688,	62.90878098	,-40.299496	,8.933198933},
//{2118.385018,	-2155.335566,	762.0924062,	69.53716964},
//{496.2250459,	-504.1370459,	177.7169226,	16.77268358}





//     Q=diag([200 300 0.0001 140 8000 400]);
//      R=[10 0;0 0.15];  0.6  0.4
{-359.202964,	397.65559	,-183.4659451	-9.252296351},
{-99.12040614,	97.04556449,	-41.57919448	-4.548868377},
{-0.030427761	,0.032580829,	-0.011951694	-0.008476794},
{-19.10293606	,20.46465792,	-7.509095915	-5.376363578},
{-627.9691634	,801.3233705,	-389.8364309	,82.64417198},
{-144.8817143	,184.7332059,	-88.97465326	,18.89650291},
{80.30150318	,34.66116579,	-83.13569702	,34.88258817},
{-126.5247056	,168.4034766,	-86.05958359	,18.0753911},
{-0.042160492	,0.084246224,	-0.053327359	,0.011535321},
{-26.98934402	,53.6293217	,-33.86148938	,7.315433646},
{1750.124365	,-1772.368366,	623.1480716,	71.29439989},
{409.048672	,-413.4477595	,144.7765271	,17.07569358}


	

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

	leg_left.leg_pid.Kp=800;	  leg_left.leg_pid.Ki=0;    leg_left.leg_pid.Kd=20000;   leg_left.leg_pid.max_iout=0;    leg_left.leg_pid.max_out=200.0f;
	leg_left.turn_pid.Kp=0.4f; 	leg_left.turn_pid.Ki=0; 	leg_left.turn_pid.Kd=0.1f; 	leg_left.turn_pid.max_iout=0; 	leg_left.turn_pid.max_out=10;
	leg_left.theta_error_pid.Kp=40; 	leg_left.theta_error_pid.Ki=0; 	leg_left.theta_error_pid.Kd=0.0f; 	leg_left.theta_error_pid.max_out=10; 	leg_left.theta_error_pid.max_iout=0;
	leg_left.roll_pid.Kp=5.0f;	  leg_left.roll_pid.Ki=0.0f;    leg_left.roll_pid.Kd=-0.5f;   leg_left.roll_pid.max_iout=0;    leg_left.roll_pid.max_out=40.0f;//40
	
	pid_init(&leg_left.leg_pid,PID_POSITION,leg_left.leg_pid.Kp,leg_left.leg_pid.Ki,leg_left.leg_pid.Kd,leg_left.leg_pid.max_out,leg_left.leg_pid.max_iout);	
	pid_init(&leg_left.turn_pid,PID_POSITION,leg_left.turn_pid.Kp,leg_left.turn_pid.Ki,leg_left.turn_pid.Kd,leg_left.turn_pid.max_out,leg_left.turn_pid.max_iout);		
	pid_init(&leg_left.theta_error_pid,PID_POSITION,leg_left.theta_error_pid.Kp,leg_left.theta_error_pid.Ki,leg_left.theta_error_pid.Kd,leg_left.theta_error_pid.max_out,leg_left.theta_error_pid.max_iout);			
	pid_init(&leg_left.roll_pid,PID_POSITION,leg_left.roll_pid.Kp,leg_left.roll_pid.Ki,leg_left.roll_pid.Kd,leg_left.roll_pid.max_out,leg_left.roll_pid.max_iout);		
	
}

void chassis_R_init()
{	
	leg_right.right_leg_pid.Kp=800;	  leg_right.right_leg_pid.Ki=0;    leg_right.right_leg_pid.Kd=20000;   leg_right.right_leg_pid.max_iout=0;    leg_right.right_leg_pid.max_out=200.0f;
	
	pid_init(&leg_right.right_leg_pid,PID_POSITION,leg_right.right_leg_pid.Kp,leg_right.right_leg_pid.Ki,leg_right.right_leg_pid.Kd,leg_right.right_leg_pid.max_out,leg_right.right_leg_pid.max_iout);
	
}


void chassis_L_task()
{
		Time_L_RUN=Time_Add;
		Time_L_Delta = Time_L_RUN-Time_L_Last;
		Time_L_Last=Time_L_RUN;
	
	
		Kalaman_feedback(&Kalman0,(float)Time_L_Delta/10000.0f,M1505B_data[0].speed_rpm*0.01047197f*M1505B_R,MPU9250_Real_Data.Accel_X*arm_cos_f32(Angular_Handler.ROLL*TheattoRad));

		err_speed1=M1505B_data[0].speed_rpm*0.01047197f*M1505B_R- Kalman0.xhat_data[0];				//用于查看在未打滑状态下卡尔曼滤波器的准确度	

		if(input_mode==stop_mode)
		{
			leg_left.jump_flag=0;
			leg_left.jump_time=0;
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
		
		leg_left.theta_error_T=PID_Calc(&leg_left.theta_error_pid,leg_right.theta-leg_left.theta,0);
		
		for(int j=0;j<12;j++)
		{
			leg_left.LQR_K[j]=Poly_Coefficient[j][0]*leg_left.L0*leg_left.L0*leg_left.L0+Poly_Coefficient[j][1]*leg_left.L0*leg_left.L0+Poly_Coefficient[j][2]*leg_left.L0+Poly_Coefficient[j][3];
		}
				
		if(leg_left.v_set>0.1f||(leg_right.leg_flag==1&&leg_left.leg_flag==1))
		{
			leg_left.x_filter=0;
		}
		else if(leg_left.recover_flag==1)
		{
			leg_left.x_filter=leg_left.x_filter+Kalman0.xhat_data[0]*Chassis_task_Time;
		}
		leg_left.v_error=-Kalman0.xhat_data[0]-leg_left.v_set;
		leg_left.v_error=fp_Limit_mouse(leg_left.v_error,-2.5f,2.5f);
		
		leg_left.wheel_T=leg_left.LQR_K[0]*(leg_left.theta-0.0f)        //负为正向
										+leg_left.LQR_K[1]*(leg_left.d_theta_filter*wheel_theta_K-0.0f)
//										 -leg_left.LQR_K[2]*(leg_left.x_filter-0)		
										 +leg_left.LQR_K[3]*(leg_left.v_error)                  //-
										 +leg_left.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
									   +leg_left.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;
		
				 leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)      //+
										 -leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f)        //-
////		
  									 -leg_left.LQR_K[9]*(leg_left.v_error)               //+
										 -leg_left.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad         //-
										 -leg_left.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;               //-
		leg_left.Tp=leg_left.Tp-leg_left.theta_error_T;
		
	//9025电机直径89mm		

//		if(rc_ctrl.rc.ch[4]>400)
//		{
//			leg_left.jump_flag=1;
//		}	
//			
//		if(leg_left.jump_flag==1&&leg_left.jump_time==0)
//		{
//			leg_left.jump_time=jump_time1;
//			while(leg_left.jump_time--)
//			{
//				leg_left.L0_set=0.4f;
//				pid_reset(&leg_left.leg_pid,1200.0f,0,0);
//			}
//			if(leg_left.jump_time==-1)
//			{
//				leg_left.jump_flag=2;
//				leg_left.jump_time=jump_time2;
//			}
//		}
//		if(leg_left.jump_flag==2)
//		{			
//			while(leg_left.jump_time--)
//			{
//				leg_left.L0_set=0.15f;
//				pid_reset(&leg_left.leg_pid,1200.0f,0,0);
//			}
//			if(leg_left.jump_time==-1)
//			{
//				leg_left.jump_flag=0;
//				leg_left.jump_time=0;
//			}
//		}
//		if(leg_left.jump_flag==0)
//		{
//			pid_reset(&leg_left.leg_pid,500.0f,0,20000.0f);
//				
//		}
		leg_left.L0_set=0.20f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*1.8f;//-0.20f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);	
		leg_left.L0_set=fp_Limit_mouse(leg_left.L0_set,0.09f,0.36f);

		leg_left.ROLL_T=turn_PID_Calc(&leg_left.roll_pid,0,Angular_Handler.Pitch+pitch_offect,Angular_Handler.V_Y);
//		if(input_mode==remote_mode)
//		{
//			pid_reset(&leg_left.leg_pid,800,0,20000);
//			pid_reset(&leg_right.right_leg_pid,800,0,20000);			
//		}
//		else if(input_mode==spin_mode)
//		{
//			pid_reset(&leg_left.leg_pid,800,0,5000);	
//			pid_reset(&leg_right.right_leg_pid,800,0,5000);			
//		}
		leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set)+leg_left.ROLL_T; 		  //L0 (0.15~0.4)  
//		leg_left.F0=all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set);
			
		leg_left.j11= (L1*arm_sin_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j12= (L1*arm_cos_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		leg_left.j21= (L4*arm_sin_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j22= (L4*arm_cos_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		
		leg_left.leg_T[0]=leg_left.j11*leg_left.F0+leg_left.j12*leg_left.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_left.leg_T[1]=leg_left.j21*leg_left.F0+leg_left.j22*leg_left.Tp;//Tp为沿中心轴的力矩 
		leg_left.leg_T[0]=-fp_Limit_mouse(leg_left.leg_T[0],-30.0f,30.0f);
		leg_left.leg_T[1]=-fp_Limit_mouse(leg_left.leg_T[1],-30.0f,30.0f);

		leg_right.v_set=leg_left.v_set=rc_ctrl.rc.ch[1]/660.0f*2.0f;
		leg_left.turn_T=turn_PID_Calc(&leg_left.turn_pid,Angular_Handler.YAW,leg_left.yaw_set,Angular_Handler.V_Z);
		leg_left.wheel_T=leg_left.wheel_T+leg_left.turn_T;		
/****************离地检测*****************/			
		leg_left.un_F0=(leg_left.j22*DM_data[2].torque-leg_left.j12*DM_data[3].torque)/(leg_left.j11*leg_left.j22-leg_left.j12*leg_left.j21);
		leg_left.un_Tp=(-leg_left.j21*DM_data[2].torque+leg_left.j11*DM_data[3].torque)/(leg_left.j11*leg_left.j22-leg_left.j12*leg_left.j21);
		
		leg_left.F1=leg_left.un_F0*arm_cos_f32(leg_left.theta);
		leg_left.F2=leg_left.un_Tp*arm_sin_f32(leg_left.theta)/leg_left.L0;
		leg_left.FN=leg_left.un_F0*arm_cos_f32(leg_left.theta)-leg_left.un_Tp*arm_sin_f32(leg_left.theta)/leg_left.L0+wheel_mg;   //支持力		
		if(leg_left.recover_flag==1&&leg_left.FN<30)
		{
				leg_left.leg_flag=1;
		}
		else
		{
				leg_left.leg_flag=0;
		}
//		if(leg_left.recover_flag==1&&leg_right.leg_flag==1&&leg_left.leg_flag==1)
//		{
//				leg_left.v_set=0;
//				leg_left.wheel_T=0;
//				leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)-leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f);				//-
//		}		
/****************离地检测*****************/			
		leg_left.wheel_T=fp_Limit_mouse(leg_left.wheel_T*744.7f,-16384.0f,16384.0f);
		if(input_mode==remote_mode||input_mode==spin_mode)
		{
			if(last_input_mode==init_mode||last_input_mode==spin_mode)
			{
				leg_left.yaw_set=leg_left.last_yaw;
			}
			leg_left.yaw_set+=rc_ctrl.rc.ch[0]/660.0f/10.0f*3.0f;
//			Can_M1505b_I();
//				Can_9025_Send1(0xA1,0);
//				vTaskDelay(1);
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
				DM_MIT(0x12,0);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x13,0);				
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
			leg_right.jump_flag=0;
			leg_right.recover_flag=0;
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

		for(int j=0;j<12;j++)
		{
			leg_right.LQR_K[j]=Poly_Coefficient[j][0]*leg_right.L0*leg_right.L0*leg_right.L0+Poly_Coefficient[j][1]*leg_right.L0*leg_right.L0+Poly_Coefficient[j][2]*leg_right.L0+Poly_Coefficient[j][3];
		}
	//9025电机直径89mm	
//		leg_right.x_set=leg_right.x_set+leg_right.v_set*Chassis_task_Time;
		if(leg_right.v_set>0.1f||(leg_right.leg_flag==1&&leg_left.leg_flag==1))
		{
			leg_right.x_filter=0;
		}
		else if(leg_right.recover_flag==1)
		{
			leg_right.x_filter=leg_right.x_filter+Kalman2.xhat_data[0]*Chassis_task_Time;
		}
		leg_right.v_error=Kalman2.xhat_data[0]-leg_right.v_set;
		leg_right.v_error=fp_Limit_mouse(leg_right.v_error,-2.5f,2.5f);
		
		leg_right.wheel_T=leg_right.LQR_K[0]*(leg_right.theta-0.0f)
										  +leg_right.LQR_K[1]*(leg_right.d_theta_filter*wheel_theta_K-0.0f)
//											-leg_right.LQR_K[2]*(leg_right.x_filter-0)
										  +leg_right.LQR_K[3]*(leg_right.v_error)                   //+   
										  +leg_right.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										  +leg_right.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;
		
				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)         			 //-
										 -leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f)           //-
//		
										 -leg_right.LQR_K[9]*(leg_right.v_error)                //+
										 -leg_right.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad             //+
										 -leg_right.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;	                   //-
		
		leg_right.Tp=leg_right.Tp+leg_left.theta_error_T;		
		
//		if(rc_ctrl.rc.ch[4]>400)
//		{
//			leg_right.jump_flag=1;
//		}	
//			
//		if(leg_right.jump_flag==1&&leg_right.jump_time==0)
//		{
//			leg_right.jump_time=jump_time1;
//			while(leg_right.jump_time--)
//			{
//				leg_right.L0_set=0.4f;
//				pid_reset(&leg_right.right_leg_pid,1200.0f,0,0);
//			}
//			if(leg_right.jump_time==-1)
//			{
//				leg_right.jump_flag=2;
//				leg_right.jump_time=jump_time2;
//			}
//		}
//		if(leg_right.jump_flag==2)
//		{			
//			while(leg_right.jump_time--)
//			{
//				leg_right.L0_set=0.15f;
//				pid_reset(&leg_right.right_leg_pid,1200.0f,0,0);
//			}
//			if(leg_right.jump_time==-1)
//			{
//				leg_right.jump_flag=0;
//				leg_right.jump_time=0;
//			}
//		}
//		if(leg_right.jump_flag==0)
//		{
//			pid_reset(&leg_right.right_leg_pid,500.0f,0,20000.0f);
//					
//		}
		leg_right.L0_set=0.20f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f;//+0.20f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);
		leg_right.L0_set=fp_Limit_mouse(leg_right.L0_set,0.09f,0.36f);
		
		leg_right.F0=-all_Mg/arm_cos_f32(leg_right.theta)-PID_Calc(&leg_right.right_leg_pid,leg_right.L0,leg_right.L0_set)-leg_left.ROLL_T;
//		leg_right.F0=0;		

		leg_right.j11= (L1*arm_sin_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j12= (L1*arm_cos_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
		leg_right.j21= (L4*arm_sin_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j22= (L4*arm_cos_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));

		leg_right.leg_T[0]=leg_right.j11*leg_right.F0+leg_right.j12*leg_right.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_right.leg_T[1]=leg_right.j21*leg_right.F0+leg_right.j22*leg_right.Tp;//Tp为沿中心轴的力矩 
		leg_right.leg_T[0]=fp_Limit_mouse(leg_right.leg_T[0],-30.0f,30.0f);
		leg_right.leg_T[1]=fp_Limit_mouse(leg_right.leg_T[1],-30.0f,30.0f);
		leg_right.wheel_T=leg_right.wheel_T-leg_left.turn_T;
/****************离地检测*****************/		
		leg_right.un_F0=(leg_right.j22*DM_data[0].torque-leg_right.j12*DM_data[1].torque)/(leg_right.j11*leg_right.j22-leg_right.j12*leg_right.j21);
		leg_right.un_Tp=(-leg_right.j21*DM_data[0].torque+leg_right.j11*DM_data[1].torque)/(leg_right.j11*leg_right.j22-leg_right.j12*leg_right.j21);
		
		leg_right.FN=-leg_right.un_F0*arm_cos_f32(leg_right.theta)-leg_right.un_Tp*arm_sin_f32(leg_right.theta)/leg_right.L0+wheel_mg;   //支持力	
		if(leg_right.recover_flag==1&&leg_right.FN<30)
		{
				leg_right.leg_flag=1;
		}
		else
		{
				leg_right.leg_flag=0;
		}
//		if(leg_right.recover_flag==1&&leg_right.leg_flag==1&&leg_left.leg_flag==1)
//		{
//				leg_right.v_set=0;
//				leg_right.wheel_T=0;
//				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)-leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f);				//-	
//		}	
/****************离地检测*****************/			
		leg_right.wheel_T=-fp_Limit_mouse(leg_right.wheel_T*744.7f,-16384.0f,16384.0f);
		if(input_mode==remote_mode||input_mode==spin_mode)
		{		
			Can_M1505b_I(leg_left.wheel_T,leg_right.wheel_T);
//			Can_9025_Send2(0xA1,0);
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
				DM_MIT(0x10,0);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x11,0);
			}
		}

}














