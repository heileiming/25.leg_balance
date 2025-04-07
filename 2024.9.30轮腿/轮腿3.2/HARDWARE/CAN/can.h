#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	
#include "chassis_task.h"
#include "iwdg.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    

//CAN初始化
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);

void CAN1_SetMsg_chassis(int32_t out1,int32_t out2,int32_t out3,int32_t out4);
void CAN1_SetMsg_6020_gimbal(int32_t out2);
void CAN2_SetMsg_shoot(int32_t out1, int32_t out2, int32_t out3);

void CAN2_SetMsg_6020(int32_t out1, int32_t out2, int32_t out3, int32_t out4);

void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power);
/************达妙**************/
void CAN1_SetMsg_damiao_pos(uint16_t id,float _pos,float _vel);
float uint_to_float(int x_int,float x_min,float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void CAN1_SetMsg_damiao_start(uint16_t id);
void CAN1_SetMsg_damiao_exit(uint16_t id);
void DM_MIT(uint16_t id,float torq);
void CAN1_SetMsg_damiao_zero(uint16_t id);
void CAN1_SetMsg_damiao_relive(uint16_t id);
/**********LK*************/
void Can_9025_Send1(int16_t control,int32_t Speed);
void Can_9025_Send2(int16_t control,int32_t Speed);
void Can_9025_stop1(void);
void Can_9025_stop2(void);
/**********LK*************/
void Can_M1505b_ID(uint8_t ID);
void Can_M1505b_I(int32_t current1,int32_t current2);
#endif

typedef struct
{
	
	uint8_t Motor_Temp;								//电机温度
	uint8_t SPEED[2];									//电机转速
	uint8_t Encoder[2];								//编码器位置
	uint8_t Iq[2];											//转矩电流
	int16_t RealSpeed;
	uint16_t RealEncoder;
	int16_t RealIq;
	uint16_t EncoderLast;
	int16_t CircleNUM;


}MF9025_typedef;

typedef struct
{
    uint16_t ecd;  //转子机械角度
    int16_t speed_rpm;     //转子转速
    int given_current;    //实际转矩电流
		int error;
		int mode;
}M1505B_typedef;


typedef struct
{
    uint16_t ecd;  //转子机械角度
    int16_t speed_rpm;     //转子转速
    int16_t given_current;    //实际转矩电流
    int32_t  all_ecd;   //编码器的值(总值)
    int32_t  count;     

    uint8_t temperate;   //电机温度
    int16_t last_ecd; 
}motor_measure_t;
//超级电容结构体
typedef struct
{
    uint16_t 	volt;
    uint16_t	power;
    uint16_t	current;
} Super_power_t;
typedef struct 
{
  uint16_t power;
	uint16_t flag;
	uint16_t buffer_power;
}Send_Data;

typedef struct
{
	int error;
	int p_int;
	int v_int;
	int t_int;
	float position;
	float velocity;
	float torque;
	
} DM_motor_measure_t;

typedef union 
{
  Send_Data TX_data;
  uint8_t Array_Tx_data[sizeof(Send_Data)];
}Tx_Union_data;

#define M1505B_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    (ptr).speed_rpm = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr).given_current = (int16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr).ecd = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]);       \
    (ptr).error = (rx_message).Data[6];                                            \
    (ptr).mode = (rx_message).Data[7];                                            \
}

#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count--;                             \
	else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count++;                        \
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]);\
    (ptr)->temperate = (rx_message).Data[6];                                            \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                        \
}

#define DM_get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
		ptr.error=(rx_message.Data[0]>>4);	              \
		ptr.p_int=(rx_message.Data[1]<<8)|rx_message.Data[2];												\
		ptr.v_int=(rx_message.Data[3]<<4)|(rx_message.Data[4]>>4);										\
		ptr.t_int=((rx_message.Data[4]&0xF)<<8)|rx_message.Data[5];										\
		ptr.position = uint_to_float(ptr.p_int, -3.1415926f, 3.1415926f, 16); 									\
		ptr.velocity = uint_to_float(ptr.v_int, -45.0f, 45.0f, 12); 									\
		ptr.torque = uint_to_float(ptr.t_int, -30.0f, 30.0f, 12);                                      \
}

















