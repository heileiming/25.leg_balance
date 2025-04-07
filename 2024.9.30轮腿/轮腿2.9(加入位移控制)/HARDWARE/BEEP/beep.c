#include "beep.h"
#include "delay.h"

void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//ʹ��GPIOFʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PC9
 GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Sound(u16 frq)

{
         u32 n;
         if(frq != 1000) //���Ƶ�ʲ�Ϊ1000��Ƶ�����������ֻ��ʱ
         {
                   n = 500000/((u32)frq);
                   BEEP = 0;
                   delay_us(n);
                   BEEP = 1;
                   delay_us(n);
         }else
                   delay_us(1000);

}
