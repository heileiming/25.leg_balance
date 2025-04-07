#include "stdio.h"
#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void pid_init(PidTypeDef *pid, u8 mode, float Kp, float Ki,float Kd,float max_out,float max_iout)
{
	pid->mode=mode;
	pid->Kp=Kp;
	pid->Ki=Ki;
	pid->Kd=Kd;
	pid->max_iout= max_iout;
	pid->max_out = max_out;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
	
}

float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];		
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;		
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);	
			  pid->Dout = pid->Kd * pid->Dbuf[0];
				pid->last_Dout=pid->Dout;
			
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

float turn_PID_Calc(PidTypeDef *pid, float ref, float set,float input)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];		
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;	
		
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];			
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);	
		pid->Dout = pid->Kd * input;
		pid->last_Dout=pid->Dout;
	
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout +pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
    return pid->out;
}

float Roll_PID_Calc(PidTypeDef *pid, float ref, float set,float input,float alpha)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];		
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;	
		
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];			
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);	
		pid->Dout = pid->Kd * input*alpha+(1.0f-alpha)*pid->last_Dout;
		pid->last_Dout=pid->Dout;
	
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout +pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
    return pid->out;
}

float PID_Calc_yaw(PidTypeDef *pid, float ref, float set)   //打符
{
		float ki;
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;	
		if(pid->error[0]<0.4f&&pid->error[0]>-0.4f)
		{
			ki=1.0f;
		}
		else 
		{
			ki=0;
			pid->Iout=0;
		}

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*ki;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);		
				
			  pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    return pid->out;
}

float PID_Calc_pitch(PidTypeDef *pid, float ref, float set)     //打符
{
		float ki;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;		
		if(pid->error[0]<0.7f&&pid->error[0]>-0.7f)
		{
			ki=1.0f;
		}
		else 
		{
			ki=0;
					pid->Iout=0;
		}

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*ki;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);		
			  pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
    return pid->out;
}

float PID_Calc_yaw_spin(PidTypeDef *pid, float ref, float set)   //打装甲板
{
		float ki;
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;	
		if(pid->error[0]<0.6f&&pid->error[0]>-0.6f)
		{
			ki=1.0f;
		}
		else 
		{
			ki=0;
			pid->Iout=0;
		}

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*ki;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);		
				
			  pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    return pid->out;
}

float PID_Calc_pitch_spin(PidTypeDef *pid, float ref, float set)     //打装甲板
{
		float ki;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;		
		if(pid->error[0]<0.9f&&pid->error[0]>-0.9f)
		{
			ki=1.0f;
		}
		else 
		{
			ki=0;
					pid->Iout=0;
		}

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*ki;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);		
			  pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
    return pid->out;
}

float PID_Calc_PC(PidTypeDef *pid, float ref, float set)
{
		float ki;
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->set - pid->fdb;		
		if(pid->error[0]<0.1f&&pid->error[0]>-0.1f)
		{
			ki=1.0f;
		}
		else
		{
			ki=0;
		}
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0]*ki;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];			
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);		
				
			  pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout +pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
void pid_reset(PidTypeDef *pid, float p, float i, float d)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;

	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->out  = 0;
	
}
