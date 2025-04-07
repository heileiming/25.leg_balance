#include "beep.h"
#include "delay.h"

void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//使能GPIOF时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PC9
 GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Sound(u16 frq)

{
         u32 n;
         if(frq != 1000) //如果频率不为1000则按频率输出，否则只延时
         {
                   n = 500000/((u32)frq);
                   BEEP = 0;
                   delay_us(n);
                   BEEP = 1;
                   delay_us(n);
         }else
                   delay_us(1000);

}
