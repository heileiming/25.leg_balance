#include "led.h" 
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys.h"
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED1_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LED2_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LED3_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LED4_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED3_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED4_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);
}


void LED_Water_lamp(void)
{
	delay_xms(200);
	LED1(0); LED2(1); LED3(1); LED4(1);

	delay_xms(200);
	LED1(1); LED2(0); LED3(1); LED4(1);

	delay_xms(200);
	LED1(1); LED2(1); LED3(0); LED4(1);

	delay_xms(200);
	LED1(1); LED2(1); LED3(1); LED4(0);
	vTaskDelay(1);
}

void LASER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA,GPIO_Pin_2);
}










