/**
  **********************************(C) NCIST ************************************
  * @file       detect_task.c/h
  * @brief      �豸�����ж�����ͨ��freeRTOS�δ�ʱ����Ϊϵͳʱ�䣬�豸��ȡ���ݺ�
  *             ����DetectHook��¼��Ӧ�豸��ʱ�䣬�ڸ������ͨ���жϼ�¼ʱ����ϵͳ
  *             ʱ��֮�����жϵ��ߡ�
  *				���֡�ʣ��жϸ�ϵͳ֡���Ƿ���������Ӧ�������ݴ�����
  *				��ȡ��Ӧ��ʩ�������ʩ���ָ���ʩ��
  *				���� 3��LED��ѯ�鿴ϵͳ״̬��ͨ��LED1������ѯ����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **********************************(C) NCIST ************************************
  */
#include "detect_task.h"
#include "led.h"
#include "timer.h"
#define DETECT_TASK_INIT_TIME 57

//��ʼ���������б�
static void DetectInit(uint32_t time);
static void DetectDisplay(void);
void DetectHook(uint8_t WDG);

monitor_t monitorList[monitorLength + 1];//static

#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t DetectTaskStack;
#endif

static void DetectInit(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime priority (��Ҫ�ظ�)
    uint16_t setItem[monitorLength][3] =
    {
        {100, 40, 14}, //JUDGEMENT 0
        {80, 40, 13}, //PCDATA 1
        {60, 40, 12}, //SBUS 2
        {2, 3, 11},   //yaw 3
        {2, 3, 10},   //pitch 4
        {10, 10, 9}, //trigger 5
        {10, 10, 8}, //motor1 6
        {10, 10, 7}, //motor2 7
        {10, 10, 6},  //motor3 8
        {10, 10, 5},  //motor4 9
        {10, 10, 4}, //friction R 10
        {10, 10, 3}, //friction L 11
    };

    for (uint8_t i = 0; i < monitorLength; i++)
    {
        monitorList[i].setOfflineTime = setItem[i][0];
        monitorList[i].setOnlineTime = setItem[i][1];
        monitorList[i].Priority = setItem[i][2];
        monitorList[i].dataIsErrorFun = NULL;
        monitorList[i].solveLostFun = NULL;
        monitorList[i].solveDataErrorFun = NULL;

        monitorList[i].enable = 1;
        monitorList[i].errorExist = 1;
        monitorList[i].isLost = 1;
        monitorList[i].dataIsError = 1;
        monitorList[i].frequency = 0.0f;
        monitorList[i].newTime = time;
        monitorList[i].lastTime = time;
        monitorList[i].Losttime = time;
        monitorList[i].worktime = time;
    }

    monitorList[DBUSWDG].dataIsErrorFun = RC_data_is_error;
    monitorList[DBUSWDG].solveLostFun = NULL;
    monitorList[DBUSWDG].solveDataErrorFun = slove_data_error;

    #if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    errorList[YawGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
    errorList[PitchGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
    #endif
}
//����������
void detect_task(void  *pvParameters)
{
    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    //��ʼ��
    DetectInit(systemTime);
    //����һ��ʱ��
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while(1)
    {
        static uint8_t error_num_display = 0;
        systemTime = xTaskGetTickCount();

        error_num_display = monitorLength;
        monitorList[monitorLength].isLost = 0;
        monitorList[monitorLength].errorExist = 0;

        for (int i = 0; i < monitorLength; i++)
        {
            //δʹ�ܣ�����
            if (monitorList[i].enable == 0)
            {
                continue;
            }

            //�жϵ���
            if (systemTime - monitorList[i].newTime > monitorList[i].setOfflineTime)//�Ѿ�����
            {
                if (monitorList[i].errorExist == 0)
                {
                    //��¼�����Լ�����ʱ��
                    monitorList[i].isLost = 1;//�Ѷ�ʧ
                    monitorList[i].errorExist = 1;//�������
                    monitorList[i].Losttime = systemTime;
                }

                //�жϴ������ȼ��� �������ȼ���ߵĴ�����
                if (monitorList[i].Priority > monitorList[error_num_display].Priority)
                {
                    error_num_display = i;
                }

                //����ṩ������������н������
                if (monitorList[i].solveLostFun != NULL)
                {
                    monitorList[i].solveLostFun();
                }
            }
            else if (systemTime - monitorList[i].worktime < monitorList[i].setOnlineTime)
            {
                //�ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
                monitorList[i].isLost = 0;//δ��ʧ
                monitorList[i].errorExist = 1;//�������
            }
            else
            {
                monitorList[i].isLost = 0;//δ��ʧ

                //�ж��Ƿ�������ݴ���
                if (monitorList[i].dataIsError)
                {
                    monitorList[i].errorExist = 1;//�������
                }
                else
                {
                    monitorList[i].errorExist = 0;//���󲻴���
                }

                //����Ƶ��
                if (monitorList[i].newTime > monitorList[i].lastTime)
                {
                    monitorList[i].frequency = configTICK_RATE_HZ / (float)(monitorList[i].newTime - monitorList[i].lastTime);
                }
            }
        }

        DetectDisplay();
        vTaskDelay(10);
        #if INCLUDE_uxTaskGetStackHighWaterMark
        DetectTaskStack = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

//�豸�������ݹ��Ӻ���
void DetectHook(uint8_t WDG)
{
    monitorList[WDG].lastTime = monitorList[WDG].newTime;
    monitorList[WDG].newTime = xTaskGetTickCount();

//���¶�ʧ���
    if (monitorList[WDG].isLost)
    {
        monitorList[WDG].isLost = 0;
        monitorList[WDG].worktime = monitorList[WDG].newTime;
    }

    //�ж������Ƿ����
    if (monitorList[WDG].dataIsErrorFun != NULL)
    {
        if (monitorList[WDG].dataIsErrorFun())	//NULL == 0
        {
            monitorList[WDG].errorExist = 1;
            monitorList[WDG].dataIsError = 1;

            if (monitorList[WDG].solveDataErrorFun != NULL)
            {
                monitorList[WDG].solveDataErrorFun();
            }
        }
        else
        {
            monitorList[WDG].dataIsError = 0;
        }
    }
    else
    {
        monitorList[WDG].dataIsError = 0;
    }
}
    
uint16_t error_count[monitorLength]={0};
float time1=0,time1_count=0,time1_error_count01=0,action=0;;
static void DetectDisplay(void)
{
//	static uint8_t last_num = errorListLength + 1;
    static uint8_t i = 0 ;
//    static int time = 0;
//    static uint8_t cnt = 0;
	
    //��ˮ������ʾÿ�������ݵĴ���������
//	
//    for (i = 0; i < frictionmotorLTOE; i++)
//    {
			while(i<=frictionmotorLTOE)
			{
				i++;
				if (monitorList[i].errorExist)
				{
					error_count[i]++;  //�������
				}
			}
//		}
			for(i=0; i < monitorLength; i++)
			{
				if (monitorList[i].errorExist)
				{
					error_count[i]++;  //�������
				}
			}
			
			if(monitorList[TX2DataWGD].errorExist)
			{
				LED3(0);
//				if(error_count[TX2DataWGD] > 1000)
//				{
//					USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
//					error_count[TX2DataWGD] %= 1000;
//				}
				
				
			}
			else
			{
				LED3(1);
			}
			if(monitorList[JudgementWDG].errorExist)
			{
				LED2(0);
			}
			else
			{
				LED3(1);
			}

//	�������б�־
//        time++;

//        if (time > 256)
//        {
//            //4��ѭ����״̬��ת
//            LED1(OFF);
//            cnt = cnt + 3;
//            time = 0;
//        }

//        if(cnt == 6)
//        {
//            LED1(ON);
//            cnt = 0;
//        }
				
//���������˸
//		if(action==0) 
//		{
//		time1++;
//		if(time1>100)
//		{
//			LED4(OFF);
//			time1_count++;
//			time1=0;
//		}
//		
//		if(time1_count==2)
//		{
//			time1_count=0;
//			LED4(ON);
//			time1_error_count01++;
//		}
//	}
//		if(time1_error_count01 == error_count01) //��˸�������ȴ�
//		{
//			LED4(OFF);
//			action++;
//			if(action>=200)
//			{
//				action=0;
//				time1_error_count01=0;
//			}
//		}
		
}


//���ض�Ӧ���豸�Ƿ���ڴ���
unsigned char toe_is_error(uint8_t err)
{
    return (monitorList[err].errorExist == 1);
}
//�����б�ָ�����ݴ���
const monitor_t *getErrorListPoint(void)
{
    return monitorList;
}












