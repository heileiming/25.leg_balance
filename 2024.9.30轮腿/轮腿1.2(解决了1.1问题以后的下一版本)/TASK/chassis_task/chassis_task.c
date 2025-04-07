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
//    Q=diag([5 1 5 10 5000 1]);            %θl X  θb
//    R=[100 0;0 25]; 
//{  -91.6631 , 128.9002 , -75.2613  , -1.0099},
//{   -2.0030  ,  3.8743 ,  -5.7026  ,  0.0734},
//{   -0.5271  ,  0.5761 ,  -0.2059  , -0.1995},
//{   -1.8925  ,  2.0730 ,  -0.7807  , -0.7502},
//{  -81.2372  ,113.2622 , -62.5843  , 16.5455},
//{   -8.6705  , 12.3879 ,  -7.1045  ,  2.2246},
//{   86.2859 ,-102.5695 ,  46.1526  ,  6.4559},
//{    3.8222 ,  -4.4756 ,   1.5389  ,  0.1839},
//{   -2.8734 ,   3.6836 ,  -1.7423  ,  0.2782},
//{  -10.5821 ,  13.5632 ,  -6.4188  ,  1.0072},
//{  171.2059 ,-208.5531 ,  93.1078  , 14.1734},
//{   22.4860 , -27.6661 ,  12.5532  ,  1.3511}
	
//    Q=diag([200 300 0.0001 140 8000 400]);            %θl X  θb
//    R=[100 0;0 25]; 	
{  -98.4021 , 132.3718 , -76.1609  , -2.1375},
{   -7.9423 ,   8.9478 ,  -6.8146  , -0.9187},
{   -0.0015 ,   0.0017 ,  -0.0007  , -0.0009},
{   -1.8162 ,   2.0670 ,  -0.7966  , -1.0805},
{  -73.3980 , 106.4094 , -62.8268  , 17.9772},
{  -11.4078 ,  16.5473 ,  -9.6950  ,  3.0318},
{  144.9775 ,-152.9744 ,  57.0189  ,  6.4718},
{  -14.0932 ,  20.2947 , -11.8771  ,  3.3020},
{   -0.0030 ,   0.0059 ,  -0.0041  ,  0.0009},
{   -3.5139 ,   7.0450 ,  -4.9104  ,  1.0757},
{  193.0386 ,-229.3367 , 101.7093  , 15.5760},
{   32.4124 , -38.2632 ,  16.7056  ,  2.6187}


//    Q=diag([200 300 0.0001 140 8000 400]);            %θl X  θb
//    R=[10 0;0 2.5]; 	
//{ -153.2831,  196.9165, -109.8414,   -5.8233},
//{  -31.2055,   34.9753,  -19.8811,   -2.8862},
//{   -0.0027,    0.0038,   -0.0019,   -0.0028},
//{   -3.1328,    4.4764,   -2.2513,   -3.3369},
//{ -147.5236,  215.4259, -127.9570,   37.4449},
//{  -27.0163,   39.9552,  -23.7568,    7.3726},
//{  179.6461, -167.6211,   41.4961,   15.2247},
//{  -33.5695,   53.0215,  -34.3163,   10.4194},
//{    0.0024,    0.0056,   -0.0082,    0.0029},
//{    2.7935,    6.7009,   -9.7410,    3.4707},
//{  413.2845, -488.9562,  213.0226,   33.2458},
//{   81.8999,  -96.6987,   41.8650,    6.6453}	
	

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

	leg_left.leg_pid.Kp=500;	  leg_left.leg_pid.Ki=0;    leg_left.leg_pid.Kd=3000;   leg_left.leg_pid.max_iout=0;    leg_left.leg_pid.max_out=100.0f;
	leg_left.turn_pid.Kp=0.5; 	leg_left.turn_pid.Ki=0; 	leg_left.turn_pid.Kd=0.1; 	leg_left.turn_pid.max_iout=0; 	leg_left.turn_pid.max_out=10;
	leg_left.theta_error_pid.Kp=40; 	leg_left.theta_error_pid.Ki=0; 	leg_left.theta_error_pid.Kd=0; 	leg_left.theta_error_pid.max_out=6; 	leg_left.theta_error_pid.max_iout=0;
	leg_left.roll_pid.Kp=3;	  leg_left.roll_pid.Ki=0;    leg_left.roll_pid.Kd=-2;   leg_left.roll_pid.max_iout=0;    leg_left.roll_pid.max_out=50.0f;
	
	pid_init(&leg_left.leg_pid,PID_POSITION,leg_left.leg_pid.Kp,leg_left.leg_pid.Ki,leg_left.leg_pid.Kd,leg_left.leg_pid.max_out,leg_left.leg_pid.max_iout);	
	pid_init(&leg_left.turn_pid,PID_POSITION,leg_left.turn_pid.Kp,leg_left.turn_pid.Ki,leg_left.turn_pid.Kd,leg_left.turn_pid.max_out,leg_left.turn_pid.max_iout);		
	pid_init(&leg_left.theta_error_pid,PID_POSITION,leg_left.theta_error_pid.Kp,leg_left.theta_error_pid.Ki,leg_left.theta_error_pid.Kd,leg_left.theta_error_pid.max_out,leg_left.theta_error_pid.max_iout);			
	pid_init(&leg_left.roll_pid,PID_POSITION,leg_left.roll_pid.Kp,leg_left.roll_pid.Ki,leg_left.roll_pid.Kd,leg_left.roll_pid.max_out,leg_left.roll_pid.max_iout);		
	
}

void chassis_R_init()
{	
	leg_right.right_leg_pid.Kp=500;	  leg_right.right_leg_pid.Ki=0;    leg_right.right_leg_pid.Kd=3000;   leg_right.right_leg_pid.max_iout=0;    leg_right.right_leg_pid.max_out=100.0f;
	
	pid_init(&leg_right.right_leg_pid,PID_POSITION,leg_right.right_leg_pid.Kp,leg_right.right_leg_pid.Ki,leg_right.right_leg_pid.Kd,leg_right.right_leg_pid.max_out,leg_right.right_leg_pid.max_iout);
	
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
		
		leg_left.v_set=rc_ctrl.rc.ch[1]/660.0f*4.0f;		
		
		leg_left.wheel_T=leg_left.LQR_K[0]*(leg_left.theta-0.0f)         //负为正向
										+leg_left.LQR_K[1]*(leg_left.d_theta_filter*0.3f-0.0f)
////										 -leg_left.LQR_K[2]*(leg_left.x_filter-leg_left.x_set)		
										 -leg_left.LQR_K[3]*(Kalman0.xhat_data[0]+leg_left.v_set)                  //-
										 +leg_left.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
									   +leg_left.LQR_K[5]*(Angular_Handler.V_X-0.0f)*TheattoRad;

				 leg_left.Tp=-leg_left.LQR_K[6]*(leg_left.theta-0.0f)      //+
										 -leg_left.LQR_K[7]*(leg_left.d_theta_filter*0.5f-0.0f)        //+
//		
  									 -leg_left.LQR_K[9]*(Kalman0.xhat_data[0]+leg_left.v_set)                //- 
										 +leg_left.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad          //-
										 -leg_left.LQR_K[11]*(Angular_Handler.V_X-0.0f)*TheattoRad;               //+
		leg_left.Tp=leg_left.Tp-leg_left.theta_error_T;
		
	//9025电机直径89mm		
		leg_left.L0_set=0.18f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f;
		leg_left.L0_set=fp_Limit_mouse(leg_left.L0_set,0.15f,0.4f);
		
		
		leg_left.F1=-all_Mg;
		leg_left.F2=-PID_Calc(&leg_left.leg_pid,leg_left.L0*arm_cos_f32(leg_left.theta),leg_left.L0_set);
		
		
		leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set)+turn_PID_Calc(&leg_left.roll_pid,0,Angular_Handler.Pitch-pitch_offect,Angular_Handler.V_Y); 		  //L0 (0.15~0.4)  
//		leg_left.F0=0;
		leg_left.j11= (L1*arm_sin_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j12= (L1*arm_cos_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		leg_left.j21= (L4*arm_sin_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j22= (L4*arm_cos_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		
		leg_left.leg_T[0]=leg_left.j11*leg_left.F0+leg_left.j12*leg_left.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_left.leg_T[1]=leg_left.j21*leg_left.F0+leg_left.j22*leg_left.Tp;//Tp为沿中心轴的力矩 
		leg_left.leg_T[0]=-fp_Limit_mouse(leg_left.leg_T[0],-12.0f,12.0f);
		leg_left.leg_T[1]=-fp_Limit_mouse(leg_left.leg_T[1],-12.0f,12.0f);	
		leg_left.turn_T=turn_PID_Calc(&leg_left.turn_pid,yaw_motor.relative_angle,0,Angular_Handler.V_Z);
		leg_left.wheel_T=leg_left.wheel_T+leg_left.turn_T;		
//		leg_left.wheel_T=leg_left.u_wheel;		
		leg_left.wheel_T=fp_Limit_mouse(leg_left.wheel_T,-4.5f,4.5f);
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
				DM_MIT(0x10,leg_left.leg_T[0]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x11,leg_left.leg_T[1]);			
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
		
		leg_right.wheel_T=leg_right.LQR_K[0]*(leg_right.theta-0.0f)
										  +leg_right.LQR_K[1]*(leg_right.d_theta_filter*0.3f-0.0f)
////											-leg_right.LQR_K[2]*(leg_right.x_filter-leg_right.x_set)
										  +leg_right.LQR_K[3]*(Kalman2.xhat_data[0]-leg_left.v_set)                   //+   
										  +leg_right.LQR_K[4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										  +leg_right.LQR_K[5]*(Angular_Handler.V_X-0.0f)*TheattoRad;

				leg_right.Tp=-leg_right.LQR_K[6]*(leg_right.theta-0.0f)           //-
										 -leg_right.LQR_K[7]*(leg_right.d_theta_filter*0.5f-0.0f)           //-
//		
/										 -leg_right.LQR_K[9]*(Kalman2.xhat_data[0]-leg_left.v_set)                //-
										 +leg_right.LQR_K[10]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad               //+
										 -leg_right.LQR_K[11]*(Angular_Handler.V_X-0.0f)*TheattoRad;	                   //-
		
		leg_right.Tp=leg_right.Tp+leg_left.theta_error_T;		
		
		leg_right.L0_set=0.18f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*2.0f;
		leg_right.L0_set=fp_Limit_mouse(leg_right.L0_set,0.15f,0.4f);
		
		leg_right.F1=-all_Mg;
		leg_right.F2=-PID_Calc(&leg_right.right_leg_pid,leg_right.L0*arm_cos_f32(leg_right.theta),leg_right.L0_set);	
		
		leg_right.F0=-all_Mg/arm_cos_f32(leg_right.theta)-PID_Calc(&leg_right.right_leg_pid,leg_right.L0,leg_right.L0_set)-turn_PID_Calc(&leg_left.roll_pid,0,Angular_Handler.Pitch-pitch_offect,Angular_Handler.V_Y);
//		leg_right.F0=0;
		leg_right.j11= (L1*arm_sin_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j12= (L1*arm_cos_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
		leg_right.j21= (L4*arm_sin_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j22= (L4*arm_cos_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));

		leg_right.leg_T[0]=leg_right.j11*leg_right.F0+leg_right.j12*leg_right.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_right.leg_T[1]=leg_right.j21*leg_right.F0+leg_right.j22*leg_right.Tp;//Tp为沿中心轴的力矩 
		leg_right.leg_T[0]=fp_Limit_mouse(leg_right.leg_T[0],-12.0f,12.0f);
		leg_right.leg_T[1]=fp_Limit_mouse(leg_right.leg_T[1],-12.0f,12.0f);
		leg_right.wheel_T=leg_right.wheel_T-leg_left.turn_T;
		leg_right.wheel_T=-fp_Limit_mouse(leg_right.wheel_T,-4.5f,4.5f);
		if(RC_Mode==CHASSIS_FOLLOW_GIMBAL)
		{		
			Can_9025_Send2(0xA1,400.0f*leg_right.wheel_T);
//			Can_9025_Send2(0xA1,0);
			vTaskDelay(1);
			if(fabs(Angular_Handler.ROLL-MPU_offect)<3.0f)
			{
				leg_right.recover_flag=1;
			}
			if(leg_right.recover_flag==1)
			{
				DM_MIT(0x12,leg_right.leg_T[1]);          //  fai1 对应 T1    fai4  对应T2
				vTaskDelay(1);
				DM_MIT(0x13,leg_right.leg_T[0]);			
			}
			else if(leg_right.recover_flag==0)
			{
				vTaskDelay(1);
			}
		}

}














