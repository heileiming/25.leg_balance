/*   system    */
#include "sys.h"
#include "delay.h"
#include "usart1.h"
/*   Freertos    */

#include "FreeRTOS.h"
#include "task.h"

/*   Ӳ��    */
#include "led.h"
#include "can.h"
#include "RemoteControl.h"
#include "timer.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "iwdg.h"
#include "timer.h"

#include "Kalaman.h"


/*   ����    */
#include "IMUTask.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "Judge_task.h"
#include "shoot_task.h"
#include "RM_Client_UI.h"
#include "TX2.h"

#include "judgement_info.h"
#include "arm_math.h"

#define IMU_TASK_PRIO		18        //IMU

#define JUDGE_TASK_PRIO 13            //����ϵͳ

#define PC_RECEIVE_TASK_PRIO		14    //�Ӿ����ݴ���

#define SEND_TASK_PRIO		15					//��PC������

#define GIMBAL_TASK_PRIO		10        //��̨

#define SHOOT_TASK_PRIO		9          //����

#define CHASSIS_L_TASK_PRIO		8         //����
#define CHASSIS_R_TASK_PRIO		8

#define UI_TASK_PRIO		6


//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_TASK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void* pvParameters);
/******��ʼ����*******/

//�������ȼ�

//�����ջ��С	
#define CHASSIS_L_TASK_SIZE 		1024  
//������
TaskHandle_t Chassis_L_Task_Handler;
//������
void Chassis_L_Task(void* pvParameters);

//�����ջ��С	
#define CHASSIS_R_TASK_SIZE 		1024  
//������
TaskHandle_t Chassis_R_Task_Handler;
//������
void Chassis_R_Task(void* pvParameters);
/******��������*******/

//�������ȼ�

//�����ջ��С	
#define IMU_TASK_SIZE 		512  
//������
TaskHandle_t IMUTask_Handler;
//������
void IMU_Task(void* pvParameters);
/******����������*******/

//�������ȼ�

//�����ջ��С	
#define GIMBAL_TASK_SIZE 		512  
//������
TaskHandle_t Gimbal_Task_Handler;
//������
void Gimbal_Task(void* pvParameters);
/******��̨����*******/

//�������ȼ�

//�����ջ��С
#define JUDGE_TASK_SIZE 512
//������
static TaskHandle_t JUDGE_TASK_Handler;
/******����ϵͳ����*******/

//�������ȼ�

//�����ջ��С	
#define SHOOT_TASK_SIZE 		512  
//������
TaskHandle_t Shoot_Task_Handler;
//������
void Shoot_Task(void* pvParameters);
/******��������*******/

//�������ȼ�

//�����ջ��С	
#define UI_TASK_SIZE 		1024  
//������
TaskHandle_t UI_Task_Handler;
//������
void UI_Task(void* pvParameters);
/******UI����*******/

//�������ȼ�

//�����ջ��С	
#define SEND_TASK_SIZE 		512  
//������
TaskHandle_t Send_Task_Handler;
//������
void Send_Task(void* pvParameters);
/******��С����TX����*******/

//�������ȼ�

//�����ջ��С	
#define PC_RECEIVE_TASK_SIZE 		512  
//������
TaskHandle_t PC_receive_Task_Handler;
//������
void PC_receive_Task(void* pvParameters);
/******��С����RX����*******/

USB_OTG_CORE_HANDLE USB_OTG_dev;

int main(void)
 {
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
		delay_init(168);//��ʼ����ʱ����
//		LASER_Init();
//		remote_control_init();
		//Device_Usart5_ENABLE_Init(115200);		
	 // TIM5_PWM_Init(20000 - 1, 84 - 1);	     ///50Hz  0.5ms~2.5ms4
		TIM5_Init(400-1,21-1);	
	
		while(MPU6500_Init());
		IMU_Calibration();
		accel_mat_init();
		ACCEL_Calibration();
		MPUHEAT_configuration();
	
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);	
		Kalman_Filter_Init(&Kalman0,2,0,2);																								//�������˲�����ʼ�����˴����������ںϹ��ƻ����ٶ�
		Kalman_Filter_Init(&Kalman2,2,0,2);
	
	
		//IWDG_Init(3,1000);	
		//USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);					
    xTaskCreate((TaskFunction_t)start_task,            //������
				(const char*)"start_task",          //��������
				(uint16_t)START_TASK_SIZE,        //�����ջ��С
				(void*)NULL,                  //���ݸ��������Ĳ���
				(UBaseType_t)START_TASK_PRIO,       //�������ȼ�
				(TaskHandle_t*)&StartTask_Handler);   //������              
				 vTaskStartScheduler();          //�����������
}
//��ʼ����������
void start_task(void* pvParameters)
{
		taskENTER_CRITICAL();           //�����ٽ���
    //����TASK1����
    xTaskCreate((TaskFunction_t)Chassis_R_Task,
        (const char*)"Chassis_Task",
        (uint16_t)CHASSIS_R_TASK_SIZE,
        (void*)NULL,
        (UBaseType_t)CHASSIS_R_TASK_PRIO,
        (TaskHandle_t*)&Chassis_R_Task_Handler);	
    xTaskCreate((TaskFunction_t)Chassis_L_Task,
        (const char*)"Chassis_Task",
        (uint16_t)CHASSIS_L_TASK_SIZE,
        (void*)NULL,
        (UBaseType_t)CHASSIS_L_TASK_PRIO,
        (TaskHandle_t*)&Chassis_L_Task_Handler);
				
				
    //��������������
    xTaskCreate((TaskFunction_t)IMU_Task,
        (const char*)"IMU_Task",
        (uint16_t)IMU_TASK_SIZE,
        (void*)NULL,
        (UBaseType_t)IMU_TASK_PRIO,
        (TaskHandle_t*)&IMUTask_Handler);
//    //����TASK2����
//    xTaskCreate((TaskFunction_t)Gimbal_Task,
//        (const char*)"Gimbal_Task",
//        (uint16_t)GIMBAL_TASK_SIZE,
//        (void*)NULL,
//        (UBaseType_t)GIMBAL_TASK_PRIO,
//        (TaskHandle_t*)&Gimbal_Task_Handler);		
//		//��������ϵͳ����			
//		xTaskCreate((TaskFunction_t )Judge_task,
//				(const char*    )"Judge_task",
//				(uint16_t       )JUDGE_TASK_SIZE,
//				(void*          )NULL,
//				(UBaseType_t    )JUDGE_TASK_PRIO,
//				(TaskHandle_t*  )&JUDGE_TASK_Handler);
//		//����
//    xTaskCreate((TaskFunction_t)Shoot_Task,
//        (const char*)"Shoot_Task",
//        (uint16_t)SHOOT_TASK_SIZE,
//        (void*)NULL,
//        (UBaseType_t)SHOOT_TASK_PRIO,
//        (TaskHandle_t*)&Shoot_Task_Handler);
//		//ui		
//    xTaskCreate((TaskFunction_t )UI_Task,
//				(const char*    )"UI_Task",
//				(uint16_t       )UI_TASK_SIZE,
//				(void*          )NULL,
//				(UBaseType_t    )UI_TASK_PRIO,
//				(TaskHandle_t*  )&UI_Task_Handler);			
//		//TX
//    xTaskCreate((TaskFunction_t)Send_Task,
//        (const char*)"Send_Task",
//        (uint16_t)SEND_TASK_SIZE,
//        (void*)NULL,
//        (UBaseType_t)SEND_TASK_PRIO,
//        (TaskHandle_t*)&Send_Task_Handler);
		//RX
//    xTaskCreate((TaskFunction_t )PC_receive_Task,
//				(const char*    )"PC_receive_Task",
//				(uint16_t       )PC_RECEIVE_TASK_SIZE,
//				(void*          )NULL,
//				(UBaseType_t    )PC_RECEIVE_TASK_PRIO,
//				(TaskHandle_t*  )&PC_receive_Task_Handler);									
			vTaskDelete(StartTask_Handler); //ɾ����ʼ����
		taskEXIT_CRITICAL();            //�˳��ٽ���
}
	

void Chassis_L_Task(void* pvParameters)
{
	chassis_L_init();
	while(1)
	{
		chassis_L_task();
		vTaskDelay(1);	
	}
}

void Chassis_R_Task(void* pvParameters)
{
	chassis_R_init();
	while(1)
	{
		chassis_R_task();
		vTaskDelay(1);	
	}
}

void Gimbal_Task(void* pvParameters)
{
	//gimbal_init();
	while(1)
	{
		//gimbal_task();
		vTaskDelay(1);
	}
}
void Shoot_Task(void* pvParameters)
{	
	//shoot_init();
	while(1)
	{
	//	shoot_task();
		vTaskDelay(1);
	}
}







