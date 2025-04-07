//����ϵͳͨѶʹ��USART1
#include "usart1.h"
QueueHandle_t TxCOM5;
QueueHandle_t RxCOM5;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
/* TX */
#define    GPIO_TX                   GPIOC
#define    GPIO_PIN_TX               GPIO_Pin_12
#define    GPIO_PINSOURCE_TX         GPIO_PinSource12
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOC

/* RX */
#define    GPIO_RX                   GPIOD
#define    GPIO_PIN_RX               GPIO_Pin_2
#define    GPIO_PINSOURCE_RX         GPIO_PinSource2
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOD
/*
*/

void Device_Usart5_ENABLE_Init( u32 bound )
{
    USART_InitTypeDef  xUsartInit;
    GPIO_InitTypeDef   xGpioInit;

    NVIC_InitTypeDef   xNvicInit;

    RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

    GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART5 );
    GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART5 );

    xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
    xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
    xGpioInit.GPIO_OType = GPIO_OType_PP;
    xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init( GPIO_TX, &xGpioInit );

    xGpioInit.GPIO_Pin = GPIO_PIN_RX;
    GPIO_Init( GPIO_RX, &xGpioInit );

    xUsartInit.USART_BaudRate            = bound;
    xUsartInit.USART_WordLength          = USART_WordLength_8b;
    xUsartInit.USART_StopBits            = USART_StopBits_1;
    xUsartInit.USART_Parity              = USART_Parity_No;
    xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;


    USART_Init( UART5, &xUsartInit );
    USART_Cmd( UART5, ENABLE );

    USART_ITConfig( UART5, USART_IT_RXNE, ENABLE  );

    {
        /* ����SEND��Ϣ���� */
        TxCOM5 = xQueueCreate(90, sizeof(u8));

        if( TxCOM5 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }

        /* ����REVICE��Ϣ���� */
        RxCOM5 = xQueueCreate(90, sizeof(u8));

        if( RxCOM5 == 0 ) /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        {
        }
    }

    USART_Cmd(UART5, ENABLE);

    xNvicInit.NVIC_IRQChannel                    = UART5_IRQn;
    xNvicInit.NVIC_IRQChannelPreemptionPriority  = Judge_NVIC;
    xNvicInit.NVIC_IRQChannelSubPriority         = 0;
    xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init( &xNvicInit );

}

//�������߿����жϣ��ڼ�⵽��������֮������������һ���ֽڵ�ʱ����û�н��յ����������ж�
//USART1 + DMA + IDLE��ʽ�����Խ���һ֡һ֡������
void UART5_IRQHandler(void)
{
    u8 Res;
    BaseType_t  TaskWoken;

    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        //clear the idle pending flag
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
        (void)UART5->SR;
        (void)UART5->DR;

        Res = USART_ReceiveData(UART5);	//��ȡ���յ�������
//        DetectHook(JudgementWDG);

        xQueueSendFromISR(RxCOM5, &Res, &TaskWoken);
    }
		   else if(USART_GetITStatus(UART5, USART_IT_ORE_RX) != RESET)    //����USART_SR 
    {
			    USART_ClearFlag(UART5,USART_FLAG_ORE); //��SR 
          USART_ReceiveData(UART5);       //����USART_DR 
    }
}







