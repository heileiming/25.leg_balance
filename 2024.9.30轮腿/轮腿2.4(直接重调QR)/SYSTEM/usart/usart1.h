#ifndef _BSP_USART_INIT_H_
#define _BSP_USART_INIT_H_

#include "stm32f4xx.h"                  // Device header
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"


extern QueueHandle_t TxCOM5;
extern QueueHandle_t RxCOM5;

void Device_Usart5_ENABLE_Init(u32 bound);


#endif

