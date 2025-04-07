#include "chassis_task.h"
#include "can.h"

extern DM_motor_measure_t DM_data[4];
extern MF9025_typedef MF9025[2];
extern int RC_Mode;
extern gimbal_typedef yaw_motor;

float L5=0.155f;   //AE
float L1=0.145f;   
float L2=0.295f;
float L3=0.295f;
float L4=0.145f;

/*滑动平均滤波器*////////
int silide_N=50;
float L_silide_buff[50];
float L_silide_sum=0;

float R_silide_buff[50];
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

//    Q=diag([100 50 0.0001 30 1000 80]);            %θl X  θb
//    R=[10 0;0 2]; 
//{  -89.2600 , 126.8630 , -76.8733 ,  -3.2380},
//{  -10.1454 ,  12.1844 ,  -8.9818 ,  -1.1025},
//{   -0.0026 ,   0.0035 ,  -0.0016 ,  -0.0029},
//{   -1.3968 ,   1.9114 ,  -0.8968 ,  -1.5908},
//{  -67.7545 , 104.3660 , -64.7911 ,  19.7853},
//{  -12.5105 ,  19.3919 , -11.9187 ,   3.8552},
//{  114.5516 ,-123.2273 ,  42.3825 ,  10.3703},
//{  -15.2635 ,  24.8099 , -16.2959 ,   4.9381},
//{    0.0000 ,   0.0086 ,  -0.0099 ,   0.0030},
//{   -0.0035 ,   4.7357 ,  -5.4476 ,   1.6263},
//{  190.4478 ,-244.8882 , 115.9151 ,  15.9879},
//{   39.7772 , -50.4641 ,  23.2483 ,   3.3404},


//   Q=diag([300 100 0.0001 200 10000 80]);            %θl X  θb
//    R=[10 0;0 2];
{ -109.0302 , 154.2966 ,-100.4155 ,  -5.0126},
{  -11.9888 ,  13.8780 , -13.9612 ,  -1.4960},
{   -0.0040 ,   0.0057 ,  -0.0029 ,  -0.0026},
{   -5.5945 ,   8.0552 ,  -4.1556 ,  -3.6757},
{ -116.9689 , 189.9655 ,-123.7438 ,  40.0904},
{  -11.5825 ,  18.5362 , -11.9785 ,   4.8610},
{  143.6489 ,-143.5542 ,  36.2074 ,  18.3798},
{  -10.1913 ,  21.0942 , -17.1916 ,   6.9256},
{   -0.0024 ,   0.0116 ,  -0.0117 ,   0.0042},
{   -3.4655 ,  16.4292 , -16.5794 ,   5.9487},
{  416.1948 ,-534.2141 , 250.3786 ,  37.6452},
{   40.4940 , -53.9045 ,  26.8041 ,   3.2194}


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

	leg_left.leg_pid.Kp=500;	  leg_left.leg_pid.Ki=0;    leg_left.leg_pid.Kd=20000;   leg_left.leg_pid.max_iout=0;    leg_left.leg_pid.max_out=200.0f;
	leg_left.turn_pid.Kp=0.5f; 	leg_left.turn_pid.Ki=0; 	leg_left.turn_pid.Kd=0.1f; 	leg_left.turn_pid.max_iout=0; 	leg_left.turn_pid.max_out=3;
	leg_left.theta_error_pid.Kp=40; 	leg_left.theta_error_pid.Ki=0; 	leg_left.theta_error_pid.Kd=5; 	leg_left.theta_error_pid.max_out=10; 	leg_left.theta_error_pid.max_iout=0;
	leg_left.roll_pid.Kp=5;	  leg_left.roll_pid.Ki=0;    leg_left.roll_pid.Kd=-1.5f;   leg_left.roll_pid.max_iout=0;    leg_left.roll_pid.max_out=40.0f;//40
	
	pid_init(&leg_left.leg_pid,PID_POSITION,leg_left.leg_pid.Kp,leg_left.leg_pid.Ki,leg_left.leg_pid.Kd,leg_left.leg_pid.max_out,leg_left.leg_pid.max_iout);	
	pid_init(&leg_left.turn_pid,PID_POSITION,leg_left.turn_pid.Kp,leg_left.turn_pid.Ki,leg_left.turn_pid.Kd,leg_left.turn_pid.max_out,leg_left.turn_pid.max_iout);		
	pid_init(&leg_left.theta_error_pid,PID_POSITION,leg_left.theta_error_pid.Kp,leg_left.theta_error_pid.Ki,leg_left.theta_error_pid.Kd,leg_left.theta_error_pid.max_out,leg_left.theta_error_pid.max_iout);			
	pid_init(&leg_left.roll_pid,PID_POSITION,leg_left.roll_pid.Kp,leg_left.roll_pid.Ki,leg_left.roll_pid.Kd,leg_left.roll_pid.max_out,leg_left.roll_pid.max_iout);		
	
}

void chassis_R_init()
{	
	leg_right.right_leg_pid.Kp=500;	  leg_right.right_leg_pid.Ki=0;    leg_right.right_leg_pid.Kd=20000;   leg_right.right_leg_pid.max_iout=0;    leg_right.right_leg_pid.max_out=200.0f;
	
	pid_init(&leg_right.right_leg_pid,PID_POSITION,leg_right.right_leg_pid.Kp,leg_right.right_leg_pid.Ki,leg_right.right_leg_pid.Kd,leg_right.right_leg_pid.max_out,leg_right.right_leg_pid.max_iout);
	
}

void kalman_task()
{
	
}

void chassis_L_task()
{
		Time_L_RUN=Time_Add;
		Time_L_Delta = Time_L_RUN-Time_L_Last;
		Time_L_Last=Time_L_RUN;

		Kalaman_feedback(&Kalman0,(float)Time_L_Delta/10000.0f,MF9025[0].RealSpeed*TheattoRad*LK_9025_R,MPU9250_Real_Data.Accel_X*arm_cos_f32(Angular_Handler.ROLL*TheattoRad));

		err_speed1=MF9025[0].RealSpeed*TheattoRad*LK_9025_R- Kalman0.xhat_data[0];				//用于查看在未打滑状态下卡尔曼滤波器的准确度	
	
		if(RC_Mode==CHASSIS_RELAX)
		{
			leg_left.recover_flag=0;
			CAN1_SetMsg_damiao_exit(0x10);		
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_exit(0x11);	
			vTaskDelay(1);				
			Can_9025_Send1(0x80,0);
		}
		else if(RC_Mode==CHASSIS_INIT)
		{				
			CAN1_SetMsg_damiao_start(0x10);		
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_start(0x11);	
			vTaskDelay(1);
			Can_9025_Send1(0x88,0);			
		}
		leg_left.fai1=3.1415926f+DM_data[0].position+0.41277036f;
		leg_left.fai4=DM_data[1].position-0.41277036f;
		
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
		
		leg_left.v_set=rc_ctrl.rc.ch[1]/660.0f*2.0f;		
		leg_left.v_error=-Kalman0.xhat_data[0]+leg_left.v_set;
		
		leg_left.wheel_T=leg_left.LQR_K[0]*(leg_left.theta-0.0f)         //负为正向
										+leg_left.LQR_K[1]*(leg_left.d_theta_filter*wheel_theta_K-0.0f)
////										 -leg_left.LQR_K[2]*(leg_left.x_filter-leg_left.x_set)		
										 -leg_left.LQR_K[3]*(-Kalman0.xhat_data[0]+leg_left.v_set)                  //-
										 +leg_left.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
									   +leg_left.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;

				 leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)                            //+
										 -leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f)        //-
//		
  									 +leg_left.LQR_K[9]*(-Kalman0.xhat_data[0]+leg_left.v_set)                //+
										 -leg_left.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad          //-
										 -leg_left.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;               //-
		leg_left.Tp=leg_left.Tp-leg_left.theta_error_T;
		
	//9025电机直径89mm		
		leg_left.L0_set=0.22f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f;//-0.34f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);
		leg_left.L0_set=fp_Limit_mouse(leg_left.L0_set,0.15f,0.4f);
		
		leg_left.ROLL_T=turn_PID_Calc(&leg_left.roll_pid,0,Angular_Handler.Pitch-pitch_offect,Angular_Handler.V_Y);
		leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set)+leg_left.ROLL_T; 		  //L0 (0.15~0.4)  
//		leg_left.F0=0;

		leg_left.FN=-leg_left.F0*arm_cos_f32(leg_left.theta)+leg_left.Tp*arm_sin_f32(leg_left.theta)/leg_left.L0+wheel_mg;   //支持力
			
		leg_left.j11= (L1*arm_sin_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j12= (L1*arm_cos_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		leg_left.j21= (L4*arm_sin_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j22= (L4*arm_cos_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		
		leg_left.leg_T[0]=leg_left.j11*leg_left.F0+leg_left.j12*leg_left.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_left.leg_T[1]=leg_left.j21*leg_left.F0+leg_left.j22*leg_left.Tp;//Tp为沿中心轴的力矩 
		leg_left.leg_T[0]=-fp_Limit_mouse(leg_left.leg_T[0],-20.0f,20.0f);
		leg_left.leg_T[1]=-fp_Limit_mouse(leg_left.leg_T[1],-20.0f,20.0f);
		
		leg_left.turn_T=turn_PID_Calc(&leg_left.turn_pid,yaw_motor.relative_angle,0,Angular_Handler.V_Z);
		
		leg_left.wheel_T=leg_left.wheel_T+leg_left.turn_T;		
		
		if(leg_left.recover_flag==1&&leg_left.FN<40&&fabs(leg_right.L0_set-leg_right.last_L0_set)<0.0001f)
		{
				leg_left.leg_flag=1;
		}
		else
		{
				leg_left.leg_flag=0;
		}
		if(leg_left.recover_flag==1&&leg_right.leg_flag==1&&leg_left.leg_flag==1)
		{
				leg_left.wheel_T=0;
				leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)-leg_left.LQR_K[7]*(leg_left.d_theta_filter*leg_theta_K-0.0f);				//-
			
		}		
		
		leg_left.wheel_T=fp_Limit_mouse(leg_left.wheel_T,-4.0f,4.0f);
		if(RC_Mode==CHASSIS_FOLLOW_GIMBAL)
		{
				Can_9025_Send1(0xA1,400.0f*leg_left.wheel_T);        //T<0 向前
//				Can_9025_Send1(0xA1,0);
				vTaskDelay(1);
			if(fabs(Angular_Handler.ROLL-MPU_offect)<3.0f)
			{
				leg_left.recover_flag=1;
			}
			if(leg_left.recover_flag==1)
			{
//				DM_MIT(0x10,leg_left.leg_T[0]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
//				DM_MIT(0x11,leg_left.leg_T[1]);			
			}
			else if(leg_left.recover_flag==0)
			{
				vTaskDelay(1);
			}
		}

}


void chassis_R_task()
{
		Time_R_RUN=Time_Add;
		Time_R_Delta = Time_R_RUN-Time_R_Last;
		Time_R_Last=Time_R_RUN;	

		Kalaman_feedback(&Kalman2,(float)Time_R_Delta/10000.0f,MF9025[1].RealSpeed*TheattoRad*LK_9025_R,MPU9250_Real_Data.Accel_X*arm_cos_f32(Angular_Handler.ROLL*TheattoRad));

		err_speed2=MF9025[1].RealSpeed*TheattoRad*LK_9025_R- Kalman2.xhat_data[0];
	
		if(RC_Mode==CHASSIS_RELAX)
		{				
			leg_right.recover_flag=0;
			CAN1_SetMsg_damiao_exit(0x12);
			vTaskDelay(1);
			CAN1_SetMsg_damiao_exit(0x13);		
			vTaskDelay(1);			
			Can_9025_Send2(0x80,0);
		}
		else if(RC_Mode==CHASSIS_INIT)
		{				
			CAN1_SetMsg_damiao_start(0x12);	
			vTaskDelay(1);			
			CAN1_SetMsg_damiao_start(0x13);	
			vTaskDelay(1);			
			Can_9025_Send2(0x88,0);			
		}	
		
		leg_right.fai1 = 3.1415926f-(DM_data[3].position+0.0f)+0.41277036f;
		leg_right.fai4 = -(DM_data[2].position-0.0f)-0.41277036f;//3jiadou180	

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
//		leg_right.x_filter=leg_right.x_filter+MF9025[1].RealSpeed*TheattoRad*0.0445f*Chassis_task_Time;
		leg_right.v_error=Kalman2.xhat_data[0]-leg_left.v_set;
		
		leg_right.wheel_T=leg_right.LQR_K[0]*(leg_right.theta-0.0f)
										  +leg_right.LQR_K[1]*(leg_right.d_theta_filter*wheel_theta_K-0.0f)
////											-leg_right.LQR_K[2]*(leg_right.x_filter-leg_right.x_set)
										  +leg_right.LQR_K[3]*(Kalman2.xhat_data[0]-leg_left.v_set)                   //+   
										  +leg_right.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										  +leg_right.LQR_K[5]*(Angular_Handler.V_X*wheel_v_x_K-0.0f)*TheattoRad;

				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)                                 //-
										 -leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f)           //-
//		
										 -leg_right.LQR_K[9]*(Kalman2.xhat_data[0]-leg_left.v_set)                //+
										 -leg_right.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad               //+
										 -leg_right.LQR_K[11]*(Angular_Handler.V_X*leg_v_x_K-0.0f)*TheattoRad;	                   //-
		
		leg_right.Tp=leg_right.Tp+leg_left.theta_error_T;		
		
		leg_right.last_L0_set=leg_right.L0_set;
		leg_right.L0_set=0.22f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f;//+0.34f*arm_sin_f32((Angular_Handler.Pitch-pitch_offect)*TheattoRad);
		leg_right.L0_set=fp_Limit_mouse(leg_right.L0_set,0.15f,0.4f);
		
		leg_right.F0=-all_Mg/arm_cos_f32(leg_right.theta)-PID_Calc(&leg_right.right_leg_pid,leg_right.L0,leg_right.L0_set)-leg_left.ROLL_T;
//		leg_right.F0=0;		

		leg_right.FN=-leg_right.F0*arm_cos_f32(leg_right.theta)+leg_right.Tp*arm_sin_f32(leg_right.theta)/leg_right.L0+wheel_mg;   //支持力

		leg_right.j11= (L1*arm_sin_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j12= (L1*arm_cos_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
		leg_right.j21= (L4*arm_sin_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j22= (L4*arm_cos_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));

		leg_right.leg_T[0]=leg_right.j11*leg_right.F0+leg_right.j12*leg_right.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_right.leg_T[1]=leg_right.j21*leg_right.F0+leg_right.j22*leg_right.Tp;//Tp为沿中心轴的力矩 
		leg_right.leg_T[0]=fp_Limit_mouse(leg_right.leg_T[0],-20.0f,20.0f);
		leg_right.leg_T[1]=fp_Limit_mouse(leg_right.leg_T[1],-20.0f,20.0f);
		leg_right.wheel_T=leg_right.wheel_T-leg_left.turn_T;
		if(leg_right.recover_flag==1&&leg_right.FN<40&&fabs(leg_right.L0_set-leg_right.last_L0_set)<0.0001f)
		{
				leg_right.leg_flag=1;
		}
		else
		{
				leg_right.leg_flag=0;
		}
		if(leg_right.recover_flag==1&&leg_right.leg_flag==1&&leg_left.leg_flag==1)
		{
				leg_right.wheel_T=0;
				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)-leg_right.LQR_K[7]*(leg_right.d_theta_filter*leg_theta_K-0.0f);				//-	
		}		
		leg_right.wheel_T=-fp_Limit_mouse(leg_right.wheel_T,-4.0f,4.0f);
		if(RC_Mode==CHASSIS_FOLLOW_GIMBAL)
		{		
			Can_9025_Send2(0xA1,400.0f*leg_right.wheel_T);
//			Can_9025_Send2(0xA1,100);
			vTaskDelay(1);
			if(fabs(Angular_Handler.ROLL-MPU_offect)<3.0f)
			{
				leg_right.recover_flag=1;
			}
			if(leg_right.recover_flag==1)
			{
//				DM_MIT(0x12,leg_right.leg_T[1]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
//				DM_MIT(0x13,leg_right.leg_T[0]);			
			}
			else if(leg_right.recover_flag==0)
			{
				vTaskDelay(1);
			}
		}

}














