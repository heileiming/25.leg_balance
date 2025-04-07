#include "TX2.h"

//PC_Ctrl_Union_t PcData;
Send_Tx2_t TX_vision_Mes;
extern ext_game_robot_status_t		GameRobotStat;				//0x0201
extern  int input_mode;

float last_YAW=0,last_PITCH=0;
int last_mouse_r=0;
int BB_Flag=0;

void Send_Task(void *pvParameters)
{		
	TX_vision_Mes.mode=0;
	while(1)
	{
		if(IF_KEY_PRESSED_B&&!B_Flag&&rc_ctrl.key.v)
		{
				B_Flag = 1;
		}				
		if(B_Flag==1&&!IF_KEY_PRESSED_B)
		{
				BB_Flag++;
				BB_Flag%=2;
				B_Flag=0;
		}
		if(GameRobotStat.robot_id > 10)  //蓝色
		{
			TX_vision_Mes.color=BLUE;
		}
		else
		{
			TX_vision_Mes.color=RED;
		}
			TX_vision_Mes.pitch=Angular_Handler.ROLL;			
			TX_vision_Mes.yaw=-Angular_Handler.YAW;	
			if(rc_ctrl.mouse.press_r==1)
			{
				if(rc_ctrl.mouse.press_r!=last_mouse_r)
				{
					last_YAW=TX_vision_Mes.yaw;
					last_PITCH=TX_vision_Mes.pitch;
				}
				TX_vision_Mes.pitch=Angular_Handler.ROLL-last_PITCH;			
				TX_vision_Mes.yaw=-Angular_Handler.YAW-last_YAW;				
				TX_vision_Mes.shoot_speed=0;
			}	
			else if(rc_ctrl.mouse.press_r==0)
			{
				TX_vision_Mes.shoot_speed=1;				
			}
//			if(GG_Flag==1||input_mode==aim_mode)
//			{
//				TX_vision_Mes.mode=9;			//打符
//			}
//			else if(GG_Flag==0)
//			{
//				if(BB_Flag)
//				{
//					TX_vision_Mes.mode=3; 						//反陀螺自动开火
//				}
//				else if(BB_Flag==0)
//				{				
//					TX_vision_Mes.mode=1;     //反陀螺手动开火
//				}
//			}
			if(input_mode==spin_mode)
			{
				TX_vision_Mes.pitch=Angular_Handler.ROLL-last_PITCH;			
				TX_vision_Mes.yaw=-Angular_Handler.YAW-last_YAW;	
				TX_vision_Mes.mode=1;  
				TX_vision_Mes.shoot_speed=0;				
			}
			last_mouse_r=rc_ctrl.mouse.press_r;			
		Send_to_PC(USART6, &TX_vision_Mes);	
		
		vTaskDelay(1);
		  
	}

}


//上位机通信函数
void PC_receive_Task(void *pvParameters)
{
	TickType_t xLastWakeTime_t = xTaskGetTickCount();			
	while(1)
	{
		//memcpy(PcData.PcDataArray,USB_USART_RX_BUF,MINIPC_FRAME_LENGTH);
//		if(PcData.PcDate.angle_pitch<-Angular_Handler.Pitch+60.0f && PcData.PcDate.angle_pitch>-Angular_Handler.Pitch-60.0f	&&	PcData.PcDate.angle_yaw<-Angular_Handler.YAW+60.0f	&&	PcData.PcDate.angle_yaw>-Angular_Handler.YAW-60.0f)
//		{PC_Flag=1;}
//		else
//		{
//			PC_Flag=0;
//		}
    	vTaskDelayUntil(&xLastWakeTime_t,1);	                         															 				
	}
}


static void Send_to_PC(USART_TypeDef* USARTx, Send_Tx2_t *TXmessage)
{
		unsigned char crc = 0;
    unsigned char *TX_data;
    TX_data = (unsigned char*)TXmessage;
    crc = 0x5a;//get_crc8_check_sum(TX_data, 12, 0xff); //CRC校验
    TXmessage->CRC8 = crc;
    //数据发送
    VCP_DataTx(0xa5);//帧头
   
    for (int i = 0; i < 14; i++)
    {
        VCP_DataTx(*TX_data); //数据 + crc		
        TX_data++;
    }

}


