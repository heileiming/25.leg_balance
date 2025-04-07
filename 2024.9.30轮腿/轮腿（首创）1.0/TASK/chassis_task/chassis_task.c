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
vmc_leg leg_left,leg_right;

uint64_t Time_L_Last;																								//全局定时器数值变量，用于记录上一次运行到此处时的时间，与下一次时间做差求运行时间
uint64_t Time_L_Delta;
uint64_t Time_L_RUN;

uint64_t Time_R_Last;																								//全局定时器数值变量，用于记录上一次运行到此处时的时间，与下一次时间做差求运行时间
uint64_t Time_R_Delta;
uint64_t Time_R_RUN;

float KKK=0;

//////////////////////////////////////////////////////////////////*********************************LQR参数*****************************************///////////////////////////////////////////////////////////////

float Poly_Coefficient[12][4]=
{
	
	
	
	
	//此部分经由Matlab求解得到，通过excel直接复制到代码内
//	
//{-138.8897514	,197.9679293	,-135.5545769	,-7.633661778},
//{-12.6481305	,14.11244353	,-18.11719272,	-2.316646492},
//{0.003043985	,0.365709145	,-0.389431369,	-1.29712952},
//{2.496453759	,-1.23305851,	-0.933598195	,-6.767374661},
//{-110.3005635,	175.343222	,-116.4019142,	42.56593724},
//{-8.631561255	,12.64338024	,-7.634239584	,4.128578273},
//{250.259451	,-286.8695806	,106.2635559	,15.71439537},
//{-8.751891377,	18.3627067	,-16.3253643,	8.381647611},
//{3.53761964	,-1.68808288,	-1.500118319	,1.128414316},
//{10.49973671,	0.142078721,	-11.18709833,	6.308085822},
//{307.5566706,	-411.4086516,	207.8910027,	41.26595304},
//{16.86045105,	-24.63486436,	14.22432082,	2.292140752}

{-120.566129	,161.4481752	,-96.69701536,	-6.436381579},
{-18.94266012	,22.64995315	,-16.27937632	,-2.273658761},
{0.3703679	,1.058845787	,-1.303735688	,-2.768190495},
{12.27042465	,-12.01376716	,3.551660476	,-4.878095018},
{-104.464883	,163.6418733	,-109.5499979	,38.63619905},
{-12.21423844	,16.95949616	,-10.11826868	,4.131443665},
{224.4231234	,-230.1595435	,70.68534258	,14.44022701},
{-9.502459998	,20.69488531	,-18.55640277	,8.56131456},
{12.84911861	,-6.712779298	,-3.73240478	,3.051371142},
{-13.74351341	,25.67758069	,-19.13708083	,6.462572244},
{367.9402662	,-456.6788131	,215.06942	,35.79977563},
{29.70707876	,-38.39008695	,19.46306697	,2.122241282}




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
	leg_left.leg_pid.Kp=1200;	  leg_left.leg_pid.Ki=0;    leg_left.leg_pid.Kd=30000;   leg_left.leg_pid.max_iout=0;    leg_left.leg_pid.max_out=300.0f;
	leg_left.turn_pid.Kp=0.5; 	leg_left.turn_pid.Ki=0; 	leg_left.turn_pid.Kd=0.1; 	leg_left.turn_pid.max_iout=0; 	leg_left.turn_pid.max_out=20;
	pid_init(&leg_left.leg_pid,PID_POSITION,leg_left.leg_pid.Kp,leg_left.leg_pid.Ki,leg_left.leg_pid.Kd,leg_left.leg_pid.max_out,leg_left.leg_pid.max_iout);	
	pid_init(&leg_left.turn_pid,PID_POSITION,leg_left.turn_pid.Kp,leg_left.turn_pid.Ki,leg_left.turn_pid.Kd,leg_left.turn_pid.max_out,leg_left.turn_pid.max_iout);		
	
}

void chassis_R_init()
{
	leg_right.right_leg_pid.Kp=1200;	  leg_right.right_leg_pid.Ki=0;    leg_right.right_leg_pid.Kd=30000;   leg_right.right_leg_pid.max_iout=0;    leg_right.right_leg_pid.max_out=300.0f;
	leg_right.turn_pid.Kp=0.5; 	leg_right.turn_pid.Ki=0; 	leg_right.turn_pid.Kd=0.1; 	leg_right.turn_pid.max_iout=0; 	leg_right.turn_pid.max_out=20;
	pid_init(&leg_right.right_leg_pid,PID_POSITION,leg_right.right_leg_pid.Kp,leg_right.right_leg_pid.Ki,leg_right.right_leg_pid.Kd,leg_right.right_leg_pid.max_out,leg_right.right_leg_pid.max_iout);
	pid_init(&leg_right.turn_pid,PID_POSITION,leg_right.turn_pid.Kp,leg_right.turn_pid.Ki,leg_right.turn_pid.Kd,leg_right.turn_pid.max_out,leg_right.turn_pid.max_iout);			
	
}


void chassis_L_task()
{
		Time_L_RUN=Time_Add;
		Time_L_Delta = Time_L_RUN-Time_L_Last;
		Time_L_Last=Time_L_RUN;
		if(RC_Mode==CHASSIS_RELAX)
		{
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
		leg_left.d_theta=(leg_left.theta-leg_left.last_theta)/Chassis_task_Time;                 //d_theta
		leg_left.d2_theta=(leg_left.d_theta-leg_left.last_d_theta)/Chassis_task_Time;
		for(int i=0;i<2;i++)    
		{
			for(int j=0;j<12;j++)
			{
				leg_left.LQR_K[i][j]=Poly_Coefficient[j][0]*leg_left.L0*leg_left.L0*leg_left.L0+Poly_Coefficient[j][1]*leg_left.L0*leg_left.L0+Poly_Coefficient[j][2]*leg_left.L0+Poly_Coefficient[j][3];
			}
		}
		leg_left.v_set=rc_ctrl.rc.ch[1]/660.0f;		
		leg_left.u_wheel=leg_left.LQR_K[0][0]*(leg_left.theta-0.0f)
//										 +leg_left.LQR_K[0][1]*(leg_left.d_theta-0.0f)
		
										 +leg_left.LQR_K[0][3]*(MF9025[0].RealSpeed*TheattoRad*0.0445f-leg_left.v_set)            
										 +leg_left.LQR_K[0][4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										 +leg_left.LQR_K[0][5]*(Angular_Handler.V_X-0.0f)*TheattoRad;

				 leg_left.Tp=0;//leg_left.LQR_K[1][0]*(leg_left.theta-0.0f);
//										+leg_left.LQR_K[1][1]*(leg_left.d_theta*0.1f+leg_left.last_d_theta*0.9f-0.0f)
		
// 										 +leg_left.LQR_K[1][3]*(MF9025[0].RealSpeed*0.0445f*TheattoRad-0.0f);          
//										 -leg_left.LQR_K[1][4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad;
//										 +leg_left.LQR_K[1][5]*(Angular_Handler.V_X-0.0f)*TheattoRad;
	//9025电机直径89mm		
		leg_left.L0_set=0.25f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*1.5f;
		leg_left.L0_set=fp_Limit_mouse(leg_left.L0_set,0.15f,0.4f);
//		leg_left.F0= -all_Mg/arm_cos_f32(leg_left.theta)-PID_Calc(&leg_left.leg_pid,leg_left.L0,leg_left.L0_set); 		  //L0 (0.15~0.4)  
		leg_left.F0=0;
		leg_left.j11 = (L1*arm_sin_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j12= (L1*arm_cos_f32(leg_left.fai0-leg_left.fai3)*arm_sin_f32(leg_left.fai1-leg_left.fai2))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
		leg_left.j21= (L4*arm_sin_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/arm_sin_f32(leg_left.fai3-leg_left.fai2);
		leg_left.j22= (L4*arm_cos_f32(leg_left.fai0-leg_left.fai2)*arm_sin_f32(leg_left.fai3-leg_left.fai4))/(leg_left.L0*arm_sin_f32(leg_left.fai3-leg_left.fai2));
	
		leg_left.leg_T[0]=-(leg_left.j11*leg_left.F0+leg_left.j12*leg_left.Tp);//F0为五连杆机构末端沿腿的推力 
		leg_left.leg_T[1]=-(leg_left.j21*leg_left.F0+leg_left.j22*leg_left.Tp);//Tp为沿中心轴的力矩 
		leg_left.leg_T[0]=fp_Limit_mouse(leg_left.leg_T[0],-30.0f,30.0f);
		leg_left.leg_T[1]=fp_Limit_mouse(leg_left.leg_T[1],-30.0f,30.0f);	
		leg_left.wheel_T=leg_left.u_wheel+turn_PID_Calc(&leg_left.turn_pid,yaw_motor.relative_angle,0,Angular_Handler.V_Z);		
//		leg_left.wheel_T=leg_left.u_wheel;		
		leg_left.wheel_T=fp_Limit_mouse(leg_left.wheel_T,-4.0f,4.0f);
		if(RC_Mode==CHASSIS_FOLLOW_GIMBAL)
		{
				Can_9025_Send1(0xA1,400.0f*leg_left.wheel_T);
//				Can_9025_Send1(0xA1,0);
				vTaskDelay(1);
				DM_MIT(0x10,leg_left.leg_T[0]);
				vTaskDelay(1);
				DM_MIT(0x11,leg_left.leg_T[1]);
		}

}


void chassis_R_task()
{
		Time_R_RUN=Time_Add;
		Time_R_Delta = Time_R_RUN-Time_R_Last;
		Time_R_Last=Time_R_RUN;	
		if(RC_Mode==CHASSIS_RELAX)
		{				
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
		
		leg_right.fai1 = 3.1415926f-(DM_data[3].position+2.14f)+0.41277036f;
		leg_right.fai4 = -(DM_data[2].position-2.10f)-0.41277036f;//3jiadou180	

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
		leg_right.last_d_theta=leg_right.d_theta;
		leg_right.d_theta=(leg_right.theta-leg_right.last_theta)/Chassis_task_Time;                 //d_theta
		leg_right.d2_theta=(leg_right.d_theta-leg_right.last_d_theta)/Chassis_task_Time;	

		for(int i=0;i<2;i++)    
		{
			for(int j=0;j<12;j++)
			{
				leg_right.LQR_K[i][j]=Poly_Coefficient[j][0]*leg_right.L0*leg_right.L0*leg_right.L0+Poly_Coefficient[j][1]*leg_right.L0*leg_right.L0+Poly_Coefficient[j][2]*leg_right.L0+Poly_Coefficient[j][3];
			}
		}
	//9025电机直径89mm	
		leg_right.v_set=rc_ctrl.rc.ch[1]/660.0f;
		leg_right.u_wheel=leg_right.LQR_K[0][0]*(leg_right.theta-0.0f)
//										  +leg_right.LQR_K[0][1]*(leg_right.d_theta-0.0f)
		
										  -leg_right.LQR_K[0][3]*(MF9025[1].RealSpeed*TheattoRad*0.0445f+leg_right.v_set)             
										  +leg_right.LQR_K[0][4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad
										  +leg_right.LQR_K[0][5]*(Angular_Handler.V_X-0.0f)*TheattoRad;

				leg_right.Tp=0;//leg_right.LQR_K[1][0]*(leg_right.theta-0.0f);
//										 +leg_right.LQR_K[1][1]*(leg_right.d_theta-0.0f)
		
//										 -leg_right.LQR_K[1][3]*(MF9025[1].RealSpeed*TheattoRad*0.0445f-0.0f);             
//										 -leg_right.LQR_K[1][4]*(Angular_Handler.ROLL-MPU_offect)*TheattoRad;
//										 +leg_right.LQR_K[1][5]*(Angular_Handler.V_X-0.0f)*TheattoRad;	

		leg_right.L0_set=0.25f+(rc_ctrl.rc.ch[2]/660.0f)/10.0f*1.5f;
		leg_right.L0_set=fp_Limit_mouse(leg_right.L0_set,0.15f,0.4f);
//		leg_right.F0= all_Mg/arm_cos_f32(leg_right.theta)+PID_Calc(&leg_right.right_leg_pid,leg_right.L0,leg_right.L0_set); 
		leg_right.F0=0;
		leg_right.j11 = (L1*arm_sin_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j12= (L1*arm_cos_f32(leg_right.fai0-leg_right.fai3)*arm_sin_f32(leg_right.fai1-leg_right.fai2))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
		leg_right.j21= (L4*arm_sin_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/arm_sin_f32(leg_right.fai3-leg_right.fai2);
		leg_right.j22= (L4*arm_cos_f32(leg_right.fai0-leg_right.fai2)*arm_sin_f32(leg_right.fai3-leg_right.fai4))/(leg_right.L0*arm_sin_f32(leg_right.fai3-leg_right.fai2));
	
		leg_right.leg_T[0]=leg_right.j11*leg_right.F0+leg_right.j12*leg_right.Tp;//F0为五连杆机构末端沿腿的推力 
		leg_right.leg_T[1]=leg_right.j21*leg_right.F0+leg_right.j22*leg_right.Tp;//Tp为沿中心轴的力矩 
		leg_right.leg_T[0]=fp_Limit_mouse(leg_right.leg_T[0],-30.0f,30.0f);
//		leg_right.leg_T[1]=fp_Limit_mouse(leg_right.leg_T[1],-30.0f,30.0f);
		leg_right.wheel_T=leg_right.u_wheel+turn_PID_Calc(&leg_right.turn_pid,yaw_motor.relative_angle,0,Angular_Handler.V_Z);
		leg_right.wheel_T=leg_right.u_wheel;
		leg_right.wheel_T=-fp_Limit_mouse(leg_right.wheel_T,-4.0f,4.0f);
		if(RC_Mode==CHASSIS_FOLLOW_GIMBAL)
		{		
			Can_9025_Send2(0xA1,400.0f*leg_right.wheel_T);
//			Can_9025_Send2(0xA1,0);
			vTaskDelay(1);
			DM_MIT(0x12,leg_right.leg_T[0]);
			vTaskDelay(1);
			DM_MIT(0x13,leg_right.leg_T[1]);
		}

}














