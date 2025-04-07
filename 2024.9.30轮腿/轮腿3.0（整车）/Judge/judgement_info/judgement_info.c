#include "judgement_info.h"
#include "protocol.h"
#include "string.h"
#include "Judge_task.h"
/*****************ϵͳ���ݶ���**********************/
ext_game_status_t       			GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_HP_t          		GameRobotSurvivors;			//0x0003
ext_dart_status_t					DartStatus;					//0x0004
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
ext_referee_warning_t				RefereeWarning;				//0x0104
ext_game_robot_status_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_t							BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207
projectile_allowance_t           pillData;           //0x208
/**�û�����**/
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
/****************************************************/
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
uint16_t cmd_id;

void judgement_data_handler(uint8_t *p_frame)
{
	frame_header_t *p_header = (frame_header_t*)p_frame;
	memcpy(p_header, p_frame, HEADER_LEN);

	uint16_t data_length = p_header->data_length;									//���ݳ���
		cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);						//����
	uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;			//���ݵ�ַ
  
  switch (cmd_id)
  {
   	case ID_game_state:        			//0x0001
			memcpy(&GameState, (data_addr), data_length);
		break;
		
		case ID_game_result:          		//0x0002
			memcpy(&GameResult, (data_addr), data_length);
		break;
		
		case ID_game_robot_HP:       //0x0003
			memcpy(&GameRobotSurvivors,(data_addr), data_length);
		break;
		
		case ID_event_data:    				//0x0101
			memcpy(&EventData, (data_addr), data_length);
		break;
		
		case ID_supply_projectile_action:   //0x0102
			memcpy(&SupplyProjectileAction, (data_addr), data_length);
		break;
		
		case ID_supply_projectile_booking:  //0x0103
			memcpy(&SupplyProjectileBooking,(data_addr), data_length);
		break;
		
		case ID_game_robot_state:      		//0x0201
			memcpy(&GameRobotStat,(data_addr), data_length);
		break;
		
		case ID_power_heat_data:      		//0x0202
			memcpy(&PowerHeatData,  (data_addr), data_length);
		break;
		
		case ID_game_robot_pos:      		//0x0203
			memcpy(&GameRobotPos,(data_addr), data_length);
		break;
		
		case ID_buff_musk:      			//0x0204
			memcpy(&BuffMusk,(data_addr), data_length);
		break;
		
		case ID_aerial_robot_energy:      	//0x0205
			memcpy(&AerialRobotEnergy,(data_addr), data_length);
		break;
		
		case ID_robot_hurt:      			//0x0206
			memcpy(&RobotHurt, (data_addr), data_length);
		break;

		case ID_shoot_data:      			//0x0207
			memcpy(&ShootData,(data_addr), data_length);
		break;
		case ID_projectile_allowance:    //0x208
			memcpy(&pillData,(data_addr), data_length);
		default : break;
  }
}


/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}
	
/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
}

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
		static u8 datalength,i;
		uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��			//���Է����û�����  �۲챾��
	
		determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
	
		ShowData.txFrameHeader.SOF = 0xA5;						//֡ͷ
		ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);   //�ֽڳ���
		ShowData.txFrameHeader.Seq = 0;

		memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
		append_crc8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����

		ShowData.CmdID = 0x0301;					//�ٷ��涨
	
		ShowData.dataFrameHeader.data_cmd_id = 0xD180;//�����ͻ��˵�cmd,�ٷ��̶�
		//ID�Ѿ����Զ���ȡ����
		ShowData.dataFrameHeader.sender_id 	 = Judge_Self_ID;//�����ߵ�ID
		ShowData.dataFrameHeader.receiver_id = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���

			//���Է��ô�СΪ16����					//���Է���ʵʱ��̨�Ƕ�   �������ݵ�����������Ϣ
		ShowData.clientData.data1 = 111.1f;
		ShowData.clientData.data2 = 222.2f;
		ShowData.clientData.data3 =	333.3f;
		//���������ĸ���
		if(1)							//λ��1																				
		{
			judge_led &= 0xfe;//��1λ��0,���
		}
		else
		{
			judge_led |= 0x01;//��1λ��1
		}
		
		
		if(1)//λ�ö�������
		{
			judge_led &= 0xfd;//��2λ��0,���
		}
		else
		{
			judge_led |= 0x02;//��2λ��1
		}
		
		
		if(1)//λ������
		{
			judge_led &= 0xfb;//��3λ��0,���
		}
		else 
		{
			judge_led |= 0x04;//��3λ��1
		}
		
		
		
		if(1)//λ����
		{
			judge_led &= 0xf7;//��3λ��0,���
		}
		else 
		{
			judge_led |= 0x08;//��3λ��1
		}
		
		ShowData.clientData.masks = judge_led;//0~5λ  0���,1�̵�
		
		
				//���д�����ݶ�
		memcpy(	
				CliendTxBuffer + 5, 
				(uint8_t*)&ShowData.CmdID, 
				(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
				);	
			
		append_crc16_check_sum(CliendTxBuffer,sizeof(ShowData));//д�����ݶ�CRC16У����		
		
		datalength = sizeof(ShowData); 
		
		for(i = 0;i < datalength;i++)
		{
			USART_SendData(USART1,(uint16_t)CliendTxBuffer[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		}	 
			
			
}
/**
  * @brief  �������ݸ�����
  * @param  void
  * @retval void
  * @attention  
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
		static u8 datalength,i;
		
		Send_Color = is_red_or_blue(); 				//	7���ڱ�(��)��	107���ڱ�(��)��
	
		memset(TeammateTxBuffer,0,200);
	
		CommuData.txFrameHeader.SOF = 0xA5;
		CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
		CommuData.txFrameHeader.Seq = 0;
		memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
		append_crc8_check_sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
		CommuData.CmdID = 0x0301;
	
	   
		CommuData.dataFrameHeader.sender_id = Judge_Self_ID;//�����ߵ�ID
	
		//�ж�ʲô���ӵ������  ���з���	���߾��Ƿ��͹�����Ϣ
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ��
	
							//�ڱ�    107��ɫ
		CommuData.dataFrameHeader.receiver_id = 107;//������ID
		
		
		CommuData.interactData.data[0] = 0;//���͵����� ,��С��Ҫ���������ı�������   

		memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
		append_crc16_check_sum(TeammateTxBuffer,sizeof(CommuData));
	
		datalength = sizeof(CommuData); 

		for(i = 0;i < datalength;i++)
		{
			USART_SendData(USART1,(uint16_t)TeammateTxBuffer[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		}	 	

}
	
