
#ifndef PID_H
#define PID_H
#include "sys.h"



enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
		
		float last_Dout;
		float last_set;
		
} PidTypeDef;
extern void  pid_init(PidTypeDef *pid, u8 mode, float Kp, float Ki,float Kd,float max_out,float max_iout);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
extern void pid_reset(PidTypeDef *pid, float p, float i, float d);


float Roll_PID_Calc(PidTypeDef *pid, float ref, float set,float input,float alpha);
float turn_PID_Calc(PidTypeDef *pid, float ref, float set,float input);
float PID_Calc_yaw(PidTypeDef *pid, float ref, float set);
float PID_Calc_pitch(PidTypeDef *pid, float ref, float set);
float PID_Calc_PC(PidTypeDef *pid, float ref, float set);
float PID_Calc_yaw_spin(PidTypeDef *pid, float ref, float set) ;
float PID_Calc_pitch_spin(PidTypeDef *pid, float ref, float set) ;


#endif

