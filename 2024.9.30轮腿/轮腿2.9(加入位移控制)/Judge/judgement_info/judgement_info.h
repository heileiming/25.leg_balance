/*
 *	@author	
 *	@date	2024/3/14

*/

#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx.h"                  // Device header

#define		JUDGE_24		24			
#define		JUDGE_VERSION	JUDGE_24

#if JUDGE_VERSION==JUDGE_24

//֡ͷ��ϸ����
#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16

/* data send (forward) */
/* data receive */
#define BLUE  0
#define RED   1


/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  11    ����״̬����       			����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1     �����������         			������������      
	ID: 0x0003  Byte:  28    ������Ѫ������   				1Hz����
	ID: 0x0004  Byte:  3 	 ���ڷ���״̬�����ڷ������
	ID: 0x0005  Byte:  11 	 �˹�������ս���ӳ���ͷ���״̬  1Hz ���ڷ��ͣ����ͷ�Χ�����л�����

	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı���� 
	ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ�����      �����ӷ��ͣ�10Hz 
	ID: 0X0104  Byte:  2    ���о������ݣ����淢������ 
	ID: 0X0105  Byte:  1    ���ڷ���ڵ���ʱ			1Hz ���ڷ��� 

	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz       
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0208  Byte:  2    ����������ʣ���ң�		1Hz ���ڷ���
	ID: 0x0209  Byte:  4    ������ RFID ״̬��			1Hz ���ڷ���
	ID: 0x020A				���ڻ����˿ͻ���ָ������
	ID: 0x020B				�������л���������
	ID: 0x020C				�Է������˱���ǽ���
	
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	ID: 0x0302  Byte:  n    �Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30Hz
	ID: 0x0303	Byte:  15	�ͻ���С��ͼ�������ݣ���������
	ID: 0x0304	Byte:  12	���̡������Ϣ��ͨ��ͼ�����ڷ���	���������ļ�֧�ָ�IDͨ��
	ID��0x0305	Byte�� 10	�״�վ�·���������Ϣ	__pack���ͽṹ��	����ʶ	�Ƚ�����
	
*/

//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       			= 0x0001,//����״̬����
	ID_game_result 	   				= 0x0002,//�����������
	ID_game_robot_HP       			= 0x0003,//������Ѫ������
	ID_game_dart_stauts				= 0x0004,//���ڷ���״̬
	ID_game_ICRA_buff_t				= 0x0005,//�˹�������ս���ӳ���ͷ���״̬
	
	ID_event_data  					= 0x0101,//�����¼����� 
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_supply_projectile_booking 	= 0x0103,//���ز���վԤԼ�ӵ�����
	ID_referee_warning_t			= 0x0104,//���о�����Ϣ
	
	ID_game_robot_state    			= 0x0201,//������״̬����
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������
	ID_projectile_allowance			= 0x0208,//����������ʣ����
	ID_rfid_status					= 0x0209,//������ RFID ״̬
	ID_dart_client_cmd				= 0x020A,//���ڻ����˿ͻ���ָ������
	ID_ground_robot_position		= 0x020B,//�������л���������
	ID_radar_mark_data				= 0x020C,//�Է������˱���ǽ���
	
	ID_ROBORT_COM_DATA				= 0x0301,//10HZ
//	__pack���ͽṹ��	����ʶ	�Ƚ�����
//	ID_client_map_command			= 0x0305,//�״��·���������Ϣ
	/*	0x0302		�Զ��������
		0x0303		С��ͼ�ͻ����·���Ϣ
		���Ͼ�δʹ��*/
} CmdID;

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct
{
	
	uint8_t game_type : 4;//0-3 bit����������
	uint8_t game_progress : 4;//4-7 bit����ǰ�����׶�
	uint16_t stage_remain_time;//��ǰ�׶�ʣ��ʱ�䣬��λ s
	uint64_t SyncTimeStamp;//�����˽��յ���ָ��ľ�ȷ Unix ʱ�䣬�����ض��յ���Ч�� NTP ��������ʱ����Ч

} ext_game_status_t;

/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    ������Ѫ������ */
typedef __packed struct
{
	uint16_t red_1_robot_HP;//�� 1 Ӣ�ۻ�����Ѫ����δ�ϳ��Լ�����Ѫ��Ϊ 0
	uint16_t red_2_robot_HP; //�� 2 ���̻�����Ѫ��
	uint16_t red_3_robot_HP; //�� 3 ����������Ѫ��
	uint16_t red_4_robot_HP; //�� 4 ����������Ѫ��
	uint16_t red_5_robot_HP; //�� 5 ����������Ѫ��
	uint16_t red_7_robot_HP; //�� 7 �ڱ�������Ѫ��
	uint16_t red_outpost_HP; //�췽ǰ��սѪ��
	uint16_t red_base_HP; 	 //�췽����Ѫ��
	uint16_t blue_1_robot_HP; //�� 1 Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP; //�� 2 ���̻�����Ѫ��
	uint16_t blue_3_robot_HP; //�� 3 ����������Ѫ��
	uint16_t blue_4_robot_HP; //�� 4 ����������Ѫ��
	uint16_t blue_5_robot_HP; //�� 5 ����������Ѫ��
	uint16_t blue_7_robot_HP; //�� 7 �ڱ�������Ѫ��
	uint16_t blue_outpost_HP; //����ǰ��վѪ��
	uint16_t blue_base_HP;	  //��������Ѫ��
} ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    ���ڷ���״̬ */
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;
//�˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����л�����
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3; 
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 

/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef __packed struct
{
	uint8_t reserved; 
	uint8_t supply_robot_id; //���������� ID��
	uint8_t supply_projectile_step; //�����ڿ���״̬��
	uint8_t supply_projectile_num;//��������
} ext_supply_projectile_action_t;

/* ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ����� ���󲹸�վ�������ݣ��ɲ����ӷ��ͣ����� 10Hz��*/
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 

/* ID: 0X0104  Byte: 2 		���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢������ */		//�Ѹ���2023
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
	uint8_t count;
}ext_referee_warning_t;

/*ID: 0X0105  Byte: 1		���ڷ���ڵ���ʱ��cmd_id (0x0105)*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 27    ������״̬���� */
typedef __packed struct
{
 uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct
{
 uint16_t chassis_voltage; 
  uint16_t chassis_current; 
  float chassis_power; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat;   
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float angle; //����ģ��ĳ���
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct
{
	uint8_t recovery_buff; //�����˻�Ѫ����
 uint8_t cooling_buff;   //�����˷�����ȴ����
 uint8_t defence_buff;    //�����˷�������
 uint8_t vulnerability_buff;   //�����˸���������
 uint16_t attack_buff;   //�����˹�������

}ext_buff_t;

/* ID: 0x0205  Byte:  2    ���л���������״̬���� */
typedef __packed struct
{
 uint8_t airforce_status;  //���л�����״̬
 uint8_t time_remain;     //״̬��ʣ��ʱ��
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
	/*
	  0x0 װ���˺���Ѫ��
	  0x1 ģ����߿�Ѫ��
    0x2 �����ٿ�Ѫ��
	  0x3 ��ǹ��������Ѫ��
	  0x4 �����̹��ʿ�Ѫ��
	  0x5 װ��ײ����Ѫ
	*/
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t launching_frequency;   //��Ƶ
	float  initial_speed; //������ٶ�
} ext_shoot_data_t;

/* ID: 0x0208	����Ƶ�ʣ�10Hz �ӵ�ʣ�෢���� ���ڷ��ͣ����л����˷���*/			//�Ѹ���2023
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;		//17mm ������������
 uint16_t projectile_allowance_42mm;		//42mm ������������
 uint16_t remaining_gold_coin;				//ʣ��������
}projectile_allowance_t;

/* ID: 0x0209	������ RFID ״̬*/
typedef __packed struct
{
	uint32_t rfid_status;
/*
bit 0����������� RFID ״̬��
bit 1���ߵ������ RFID ״̬��
bit 2���������ؼ���� RFID ״̬��
bit 3����������� RFID ״̬��
bit 4��ǰ�ڸ������ RFID ״̬��
bit 5����Դ������� RFID ״̬��
bit 6����Ѫ������� RFID ״̬��
bit 7�����̻����˲�Ѫ�� RFID ״̬��
*/
} ext_rfid_status_t;

/* ID: 0x020A	���ڻ����˿ͻ���ָ������*/		//�Ѹ���2023
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* ID: 0x020B	�������л���������	��λ����*/		//�Ѹ���2023
typedef __packed struct
{
 float hero_x;			//����Ӣ�ۻ�����λ�� X ������
 float hero_y;			//����Ӣ�ۻ�����λ�� Y ������
 float engineer_x;		//�������̻�����λ�� X ������
 float engineer_y;		//�������̻�����λ�� Y ������
 float standard_3_x;	//���� 3 �Ų���������λ�� X ������
 float standard_3_y;	//���� 3 �Ų���������λ�� Y ������
 float standard_4_x;	//���� 4 �Ų���������λ�� X ������
 float standard_4_y;	//���� 4 �Ų���������λ�� Y ������
 float standard_5_x;	//���� 5 �Ų���������λ�� X ������
 float standard_5_y;	//���� 5 �Ų���������λ�� Y ������
}ground_robot_position_t;

/* ID: 0x020C	�Է������˱���ǽ���	*/		//�Ѹ���2023
typedef __packed struct
{
 uint8_t mark_hero_progress;
uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;

/* ID��0x020D	�ڱ��һ�����*/		//�Ѹ���2024
typedef __packed struct
{
 uint32_t sentry_info;
	
} sentry_info_t;

/* ID��0x020E	�״�����״̬*/		//�Ѹ���2024
typedef __packed struct
{
  uint8_t radar_info;
	
} radar_info_t;


/* ID��0x0305	�����״﷢�͵�������Ϣ*/		//�Ѹ���2023
//typedef __pack struct
//{
//uint16_t target_robot_ID;		//Ŀ������� ID
//float target_position_x;		//Ŀ�� x λ�����꣬��λ m
//float target_position_y;		//Ŀ�� y λ�����꣬��λ m
//} ext_client_map_command_t;

/* 
	
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 
*/

typedef __packed struct   //�������ݽ�����Ϣ��0x0301
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[113];
}ext_student_interactive_header_data_t;

typedef struct 
{
	uint8_t	 Flag;			//֡ͷλ��
	uint16_t data_len;		//���ݳ���
	uint16_t data_cnt;		//�Լ�λ��
	uint8_t	 data;
}Judge_FLAG;

typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;
  float data1;
  float data2;
  float data3;
	u8    mask;
} client_show_data_t;

typedef __packed struct
{
	xFrameHeader		FrameHeader;
	CmdID			    CmdID;

  __packed union
	{ 
		ext_game_status_t 			game_information;
		ext_robot_hurt_t  			blood_changed_data;
		ext_shoot_data_t       		real_shoot_data;
		ext_power_heat_data_t   	real_powerheat_data;
		ext_event_data_t      		rfid_data;
		ext_game_result_t      		game_result_data;
		ext_buff_t         			get_buff_data;
		ext_game_robot_pos_t		gameRobotPos;
		client_show_data_t  		client_show_data;
	}Data;
	uint16_t		CRC16;	//��������CRCУ��
}Dateframe_t;//����֡

/* �������ݽ�����Ϣ��0x0301  */

/* 
	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	����Ƶ�ʣ����� 10Hz


	1.	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	�ֽ�ƫ���� 	��С 	˵�� 				��ע 
	0 			2 		���ݵ����� ID 		0xD180 
	2 			2 		���ߵ� ID 			��ҪУ�鷢���߻����˵� ID ��ȷ�� 
	4 			2 		�ͻ��˵� ID 		ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ��� 
	6 			4 		�Զ��帡������ 1 	 
	10 			4 		�Զ��帡������ 2 	 
	14 			4 		�Զ��帡������ 3 	 
	18 			1 		�Զ��� 8 λ���� 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;

/*-------------1. �������ݽ�����Ϣ�� 0x0301�� ����Ƶ�ʣ����� 10Hz---------
�ֽ�ƫ���� ��С ˵�� ��ע
0 2 ���ݶε�����ID
2 2 �����ߵ�ID   ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5��������ҪУ��� 1
4 2 �����ߵ�ID  ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID
6 x �������ݶ�x ���Ϊ 113
----*/

//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//֡ͷ
	uint16_t		 						CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	client_custom_data_t  					clientData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_SendClientData_t;
//�ͻ���ɾ��ͼ�� �����˼�ͨ�ţ�0x0301
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;
//ͼ������
typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
} graphic_data_struct_t;

//�ͻ��˻���һ��ͼ�� �����˼�ͨ�ţ�0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
//�ͻ��˻�������ͼ�� �����˼�ͨ�ţ�0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//�ͻ��˻������ͼ�� �����˼�ͨ�ţ�0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
//�ͻ��˻����ַ� �����˼�ͨ��
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;
//�ͻ��˻����߸�ͼ�� �����˼�ͨ��
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
//�ͻ����·���Ϣ
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} ext_robot_commands_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
/* data send (forward) */
/* data receive */


#define INFANTRY 1
#define HERO 2
#define ENGINEER 3
#define SENTINEL 4
typedef struct
{
	int16_t Voltage;
	int16_t Current;
	int16_t Supply_Num;
	int16_t ShooterHeat_17mm;
	int16_t ShooterHeat_42mm;
	int16_t Power;
	int16_t PowerBuffer;
	uint8_t level;
	uint8_t hurt_type;
}JudgementType;

typedef struct
{
	uint8_t 	graph_operate_type;
	uint8_t 	graph_type;
	uint8_t 	graph_name[5];
	uint8_t 	graph_color;
	uint8_t		graph_line_width;
	uint16_t graph_start_x;
	uint16_t graph_start_y;
	uint16_t graph_radius;
	uint16_t graph_dst_x;
	uint16_t graph_dst_y;
	uint8_t text_lengh;
	uint8_t text[30];
}Graph_Data_Type;

typedef struct
{
	int16_t 		ShootLevel;
	int16_t 		SuperCapacitorComment;
	float 					bullet_can_shoot;
	uint8_t 		State_Mask;
	uint8_t 		SuperCapacitorState;
	Graph_Data_Type Graph_Data ;
}SendToJudgementDataType;

//�����˽�����Ϣ
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//֡ͷ
	uint16_t								CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 			interactData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_CommunatianData_t;


#endif


void  judgement_data_handler(uint8_t *p_frame);
void determine_ID(void);
#endif
