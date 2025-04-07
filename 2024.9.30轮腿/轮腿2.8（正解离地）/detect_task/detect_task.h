///**
//  **********************************(C) NCIST ************************************
//  * @file       detect_task.c/h
//  * @brief      �豸�����ж�����ͨ��freeRTOS�δ�ʱ����Ϊϵͳʱ�䣬�豸��ȡ���ݺ�
//  *             ����DetectHook��¼��Ӧ�豸��ʱ�䣬�ڸ������ͨ���жϼ�¼ʱ����ϵͳ
//  *             ʱ��֮�����жϵ��ߡ�
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     Dec-26-2018     RM              1. ���
//  *
//  @verbatim
//  ==============================================================================

//  ==============================================================================
//  @endverbatim
//  **********************************(C) NCIST ************************************
//  */
#ifndef _DETECT_TASK_H_
#define _DETECT_TASK_H_

#include "rc.h"
#include "RemoteControl.h"
#include "sys.h"
///*
//*********************************************************************************************************
//* OS
//*********************************************************************************************************
//*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"

//�������Լ���Ӧ�豸˳��
enum errorList
{
    JudgementTOE = 0,
    TX2DataTOE,
    DBUSTOE,

    YawGimbalMotorTOE,
    PitchGimbalMotorTOE,
    TriggerMotorTOE,

    ChassisMotor1TOE,
    ChassisMotor2TOE,
    ChassisMotor3TOE,

    ChassisMotor4TOE,
    frictionmotorRTOE,
    frictionmotorLTOE,

    errorListLength,
};
//�������Լ���Ӧ�豸˳��
enum monitorList
{
    JudgementWDG = 0,
    TX2DataWGD,
    DBUSWDG,
    YawMotorWDG,
    PitchMotorWDG,
    TriggerWDG,
    Chassis1WDG,
    Chassis2WDG,
    Chassis3WDG,
    Chassis4WDG,
    frictionmotorRWDG,
    frictionmotorLWDG,

    monitorLength,
};

typedef __packed struct
{
    uint32_t newTime;
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;
    uint16_t setOfflineTime : 12;
    uint16_t setOnlineTime : 12;
    uint8_t  enable : 1;
    uint8_t  Priority : 4;
    uint8_t  errorExist : 1;
    uint8_t  isLost : 1;
    uint8_t  dataIsError : 1;

    float frequency;

    unsigned char (*dataIsErrorFun)(void);//�ж����ݴ��� -- �˺����� char�ͷ���ֵ
    void (*solveLostFun)(void);//�����ʧ
    void (*solveDataErrorFun)(void);//������ݴ���
} monitor_t;

//extern const monitor_t *getErrorListPoint(void);
//extern char toe_is_error(uint8_t err);
extern void DetectHook(uint8_t WDG);
//extern void detect_task(void  *pvParameters);
////static void DetectInit(uint32_t time);
#endif




