/*   system    */
#include "sys.h"
#include "delay.h"
#include "usart1.h"
/*   Freertos    */

#include "FreeRTOS.h"
#include "task.h"

/*   硬件    */
#include "led.h"
#include "can.h"
#include "RemoteControl.h"
#include "timer.h"
#include "iwdg.h"
#include "timer.h"

#include "Kalaman.h"


/*   任务    */
#include "IMUTask.h"
#include "chassis_task.h"
#include "Judge_task.h"
#include "RM_Client_UI.h"

#include "judgement_info.h"
#include "arm_math.h"

#define IMU_TASK_PRIO		18        //IMU

#define JUDGE_TASK_PRIO 13            //裁判系统

#define CHASSIS_L_TASK_PRIO		8         //底盘
#define CHASSIS_R_TASK_PRIO		8



//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_TASK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void* pvParameters);
/******开始任务*******/

//任务优先级

//任务堆栈大小	
#define CHASSIS_L_TASK_SIZE 		1024  
//任务句柄
TaskHandle_t Chassis_L_Task_Handler;
//任务函数
void Chassis_L_Task(void* pvParameters);

//任务堆栈大小	
#define CHASSIS_R_TASK_SIZE 		1024  
//任务句柄
TaskHandle_t Chassis_R_Task_Handler;
//任务函数
void Chassis_R_Task(void* pvParameters);
/******底盘任务*******/

//任务优先级

//任务堆栈大小	
#define IMU_TASK_SIZE 		512  
//任务句柄
TaskHandle_t IMUTask_Handler;
//任务函数
void IMU_Task(void* pvParameters);
/******陀螺仪任务*******/

//任务优先级


//任务优先级

//任务堆栈大小
#define JUDGE_TASK_SIZE 512
//任务句柄
static TaskHandle_t JUDGE_TASK_Handler;
/******裁判系统任务*******/


//任务优先级

//任务堆栈大小	
#define UI_TASK_SIZE 		1024  
//任务句柄
TaskHandle_t UI_Task_Handler;
//任务函数
void UI_Task(void* pvParameters);
/******UI任务*******/


int main(void)
 {
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
		delay_init(168);//初始化延时函数
//		LASER_Init();
		remote_control_init();
		//Device_Usart5_ENABLE_Init(115200);		
	 // TIM5_PWM_Init(20000 - 1, 84 - 1);	     ///50Hz  0.5ms~2.5ms4
		TIM5_Init(400-1,21-1);	
	
		while(MPU6500_Init());
		IMU_Calibration();
		accel_mat_init();
		ACCEL_Calibration();
		MPUHEAT_configuration();
	
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);	
		Kalman_Filter_Init(&Kalman0,2,0,2);																								//卡尔曼滤波器初始化，此处用于数据融合估计机体速度
		Kalman_Filter_Init(&Kalman2,2,0,2);
	
	
		//IWDG_Init(3,1000);					
    xTaskCreate((TaskFunction_t)start_task,            //任务函数
				(const char*)"start_task",          //任务名称
				(uint16_t)START_TASK_SIZE,        //任务堆栈大小
				(void*)NULL,                  //传递给任务函数的参数
				(UBaseType_t)START_TASK_PRIO,       //任务优先级
				(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
				 vTaskStartScheduler();          //开启任务调度
}
//开始任务任务函数
void start_task(void* pvParameters)
{
		taskENTER_CRITICAL();           //进入临界区
    //创建TASK1任务
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
				
				
    //创建陀螺仪任务
    xTaskCreate((TaskFunction_t)IMU_Task,
        (const char*)"IMU_Task",
        (uint16_t)IMU_TASK_SIZE,
        (void*)NULL,
        (UBaseType_t)IMU_TASK_PRIO,
        (TaskHandle_t*)&IMUTask_Handler);
//		//创建裁判系统任务			
//		xTaskCreate((TaskFunction_t )Judge_task,
//				(const char*    )"Judge_task",
//				(uint16_t       )JUDGE_TASK_SIZE,
//				(void*          )NULL,
//				(UBaseType_t    )JUDGE_TASK_PRIO,
//				(TaskHandle_t*  )&JUDGE_TASK_Handler);
//		//ui		
//    xTaskCreate((TaskFunction_t )UI_Task,
//				(const char*    )"UI_Task",
//				(uint16_t       )UI_TASK_SIZE,
//				(void*          )NULL,
//				(UBaseType_t    )UI_TASK_PRIO,
//				(TaskHandle_t*  )&UI_Task_Handler);											
			vTaskDelete(StartTask_Handler); //删除开始任务
		taskEXIT_CRITICAL();            //退出临界区
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







