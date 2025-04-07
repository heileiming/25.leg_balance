#include "can.h"
//#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

motor_measure_t m3508_data[4];
motor_measure_t pitch_data,yaw_data;
motor_measure_t m2006_data,friction_left_data,friction_right_data;
Super_power_t Super_power;
DM_motor_measure_t DM_data[4];

extern gimbal_typedef yaw_motor;
MF9025_typedef MF9025[2];
int RC_Mode;
int64_t a=0,b=0,c=0,d=0;

u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
    NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    //��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12

      //���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1); //GPIOA11����ΪCAN1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1); //GPIOA12����ΪCAN1

  //CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM = DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
    CAN_InitStructure.CAN_ABOM = ENABLE;	//����Զ����߹���	  
    CAN_InitStructure.CAN_AWUM = DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART = ENABLE;	//��ֹ�����Զ����� 
    CAN_InitStructure.CAN_RFLM = DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
    CAN_InitStructure.CAN_TXFP = DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
    CAN_InitStructure.CAN_Mode = mode;	 //ģʽ���� 
    CAN_InitStructure.CAN_SJW = tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
    CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 

        //���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber = 0;	  //������0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ 
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������0
    CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
    CAN_ITConfig(CAN1, CAN_IT_ERR, DISABLE); //��ֹ�����ж�
#if CAN1_RX0_INT_ENABLE

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =CAN1_NVIC;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    return 0;
}

u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;//����Զ����߹���
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN2, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 14;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    CAN_ITConfig(CAN2, CAN_IT_ERR, DISABLE); //��ֹ�����ж�
    CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CAN2_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return 0;

}

float uint_to_float(int x_int,float x_min,float x_max, int bits)
{
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
	
}
int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void CAN1_SetMsg_chassis(int32_t out1, int32_t out2, int32_t out3, int32_t out4)
{
    CanTxMsg TxMessage1;
    TxMessage1.StdId = 0x200;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = out1 >> 8;
    TxMessage1.Data[1] = out1;
    TxMessage1.Data[2] = out2 >> 8;
    TxMessage1.Data[3] = out2;
    TxMessage1.Data[4] = out3 >> 8;
    TxMessage1.Data[5] = out3;
    TxMessage1.Data[6] = out4 >> 8;
    TxMessage1.Data[7] = out4;
    CAN_Transmit(CAN1, &TxMessage1);
}
void CAN1_SetMsg_6020_gimbal(int32_t out2)
{
    CanTxMsg TxMessage1;
    TxMessage1.StdId = 0x1FF;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = 0;
    TxMessage1.Data[1] = 0;
    TxMessage1.Data[2] = out2>> 8;
    TxMessage1.Data[3] = out2;
    TxMessage1.Data[4] = 0;
    TxMessage1.Data[5] = 0;
    TxMessage1.Data[6] = 0;
    TxMessage1.Data[7] = 0;
    CAN_Transmit(CAN1, &TxMessage1);
}
void CAN2_SetMsg_shoot(int32_t out1, int32_t out2, int32_t out3)
{

    CanTxMsg TxMessage2;
    TxMessage2.StdId = 0x200;
    TxMessage2.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage2.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage2.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage2.Data[0] = out1 >> 8;
    TxMessage2.Data[1] = out1;
    TxMessage2.Data[2] = out2 >> 8;
    TxMessage2.Data[3] = out2;
    TxMessage2.Data[4] = out3 >> 8;
    TxMessage2.Data[5] = out3;
    TxMessage2.Data[6] = 0;
    TxMessage2.Data[7] = 0 ;
    CAN_Transmit(CAN2, &TxMessage2);
}
void CAN2_SetMsg_6020(int32_t out1, int32_t out2, int32_t out3, int32_t out4)
{

    CanTxMsg TxMessage2;
    TxMessage2.StdId = 0x1FF;
    TxMessage2.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage2.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage2.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage2.Data[0] = out1 >> 8;
    TxMessage2.Data[1] = out1;
    TxMessage2.Data[2] = out2 >> 8;
    TxMessage2.Data[3] = out2;
    TxMessage2.Data[4] = out3 >> 8;
    TxMessage2.Data[5] = out3;
    TxMessage2.Data[6] = out4 >> 8;
    TxMessage2.Data[7] = out4;
    CAN_Transmit(CAN2, &TxMessage2);
}

Tx_Union_data Can_Tx_Data;
void CAN_CMD_SUPERPOWER(int16_t power, int16_t i,uint16_t buffer_power)
{
	CanTxMsg SendCanTxMsg;
	  Can_Tx_Data.TX_data .power =power;
	  Can_Tx_Data.TX_data .flag =i;
	  Can_Tx_Data.TX_data .buffer_power =buffer_power;
    SendCanTxMsg.StdId = 0x222;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x08;
	 for(int i=0;i<sizeof(Send_Data);i++)
	{
    SendCanTxMsg.Data[i] = Can_Tx_Data.Array_Tx_data[i];
	}
    CAN_Transmit(CAN1, &SendCanTxMsg);
}

void CAN1_SetMsg_damiao_start(uint16_t id)
{
		u8 mbox;
    CanTxMsg TxMessage1;
    TxMessage1.StdId = id;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = 0xFF;
    TxMessage1.Data[1] = 0xFF;
    TxMessage1.Data[2] = 0xFF;
    TxMessage1.Data[3] = 0xFF;
    TxMessage1.Data[4] = 0xFF;
    TxMessage1.Data[5] = 0xFF;
    TxMessage1.Data[6] = 0xFF;
    TxMessage1.Data[7] = 0xFC;
		mbox= CAN_Transmit(CAN1, &TxMessage1); 
		while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok));	//�ȴ����ͽ���
}

void CAN1_SetMsg_damiao_zero(uint16_t id)
{

    CanTxMsg TxMessage1;
    TxMessage1.StdId = id;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = 0xFF;
    TxMessage1.Data[1] = 0xFF;
    TxMessage1.Data[2] = 0xFF;
    TxMessage1.Data[3] = 0xFF;
    TxMessage1.Data[4] = 0xFF;
    TxMessage1.Data[5] = 0xFF;
    TxMessage1.Data[6] = 0xFF;
    TxMessage1.Data[7] = 0xFE;
    CAN_Transmit(CAN1, &TxMessage1);
}

void CAN1_SetMsg_damiao_exit(uint16_t id)
{
		u8 mbox;
    CanTxMsg TxMessage1;
    TxMessage1.StdId = id;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = 0xFF;
    TxMessage1.Data[1] = 0xFF;
    TxMessage1.Data[2] = 0xFF;
    TxMessage1.Data[3] = 0xFF;
    TxMessage1.Data[4] = 0xFF;
    TxMessage1.Data[5] = 0xFF;
    TxMessage1.Data[6] = 0xFF;
    TxMessage1.Data[7] = 0xFD;
    mbox= CAN_Transmit(CAN1, &TxMessage1); 
		while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok));	//�ȴ����ͽ���
}

void CAN1_SetMsg_damiao_relive(uint16_t id)
{

    CanTxMsg TxMessage1;
    TxMessage1.StdId = id;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = 0xFF;
    TxMessage1.Data[1] = 0xFF;
    TxMessage1.Data[2] = 0xFF;
    TxMessage1.Data[3] = 0xFF;
    TxMessage1.Data[4] = 0xFF;
    TxMessage1.Data[5] = 0xFF;
    TxMessage1.Data[6] = 0xFF;
    TxMessage1.Data[7] = 0xFB;
    CAN_Transmit(CAN1, &TxMessage1);
}

void CAN1_SetMsg_damiao_pos(uint16_t id,float _pos,float _vel)
{
    CanTxMsg TxMessage1;
	
		uint8_t *pbuf,*vbuf;
		pbuf=(uint8_t*)&_pos;
		vbuf=(uint8_t*)&_vel;
    TxMessage1.StdId = id;
		TxMessage1.ExtId=0;
    TxMessage1.IDE = CAN_Id_Standard; //��׼ģʽ
    TxMessage1.RTR = CAN_RTR_DATA; //���͵�������
    TxMessage1.DLC = 8; //���ݳ���Ϊ 8 �ֽ�
    TxMessage1.Data[0] = *pbuf;
    TxMessage1.Data[1] = *(pbuf+1);
    TxMessage1.Data[2] = *(pbuf+2);
    TxMessage1.Data[3] = *(pbuf+3);
    TxMessage1.Data[4] = *vbuf;
    TxMessage1.Data[5] = *(vbuf+1);
    TxMessage1.Data[6] = *(vbuf+2);
    TxMessage1.Data[7] = *(vbuf+3);
    CAN_Transmit(CAN1, &TxMessage1);
}

void DM_MIT(uint16_t id,float torq)
{
		u8 mbox;
	  CanTxMsg TxMessage1;
		uint16_t tor_tmp;
//		 pos_tmp,vel_tmp,kp_tmp,kd_tmp,;
//		pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
//		vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
//		kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
//		kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
		tor_tmp = float_to_uint(torq, -54, 54, 12);

	 TxMessage1.StdId = id;
	 TxMessage1.IDE = CAN_Id_Standard;
	 TxMessage1.RTR = CAN_RTR_DATA;
	 TxMessage1.DLC = 0x08;
	 TxMessage1.Data[0] = 0;
	 TxMessage1.Data[1] = 0;
	 TxMessage1.Data[2] = 0;
	 TxMessage1.Data[3] = 0;
	 TxMessage1.Data[4] = 0;
	 TxMessage1.Data[5] = 0;
	 TxMessage1.Data[6] = tor_tmp >> 8;
	 TxMessage1.Data[7] = tor_tmp;
   mbox= CAN_Transmit(CAN1, &TxMessage1); 
	 while((CAN_TransmitStatus(CAN1, mbox)!=CAN_TxStatus_Ok));	//�ȴ����ͽ���		
}

/*
	0x91 д�������ֵ��Ϊ����������
	0x81 ���
	0x92 ��ȡ��Ȧ�Ƕ����� 1-7 �Ƕ��ֽ�
	0x94 ��ȡ��Ȧ�Ƕ����� 6-7 �Ƕ��ֽ�6��7��
	0x9c ��ȡ������¶ȡ���ѹ��ת�١�������λ�� 1 23 45 67
	0x80 ����ر����� �������״̬����յĿ�������
	0x81 ���ֹͣ���� ����������״̬
	0x88 �����������
	0xA1 ת�رջ��������� 45 ת�ص������Ƹߵ��ֽ� 								����ظ� 1�¶� 23ת�ص��� 45����ٶ� 67������λ��
	0xA2 �ٶȱջ��������� 4567�ٶȿ��� 													����ظ� 1�¶� 23ת�ص��� 45����ٶ� 67������λ��
	0xA4 λ�ÿ�������2(��Ȧ�Ƕ�)    34�ٶ������ֽ� 4567λ�ÿ����ֽ�
	0xA5 λ�ÿ�������3(��Ȧ�Ƕ�)    1ת������ 45λ�ÿ����ֽ�
	0xA6 λ�ÿ�������3(��Ȧ�Ƕ�)    1ת������ 23�ٶ������ֽ� 4567λ�ÿ����ֽ�


*/

void Can_9025_Send1(int16_t control,int32_t Speed)
{
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x141;																																												 					 // ��׼��ʶ�� 				   																																								 // ������չ��ʾ�� 
		TxMessage.IDE=CAN_Id_Standard;						 																																		 					 // ��׼֡
		TxMessage.RTR=CAN_RTR_Data;		 																																													 // ����֡
		TxMessage.DLC=0x08;		
		TxMessage.Data[0] =	control;
		
		TxMessage.Data[4]	=	Speed<<24>>24;
		TxMessage.Data[5]	=	Speed<<16>>24;
		TxMessage.Data[6]	=	Speed<< 8>>24;
		TxMessage.Data[7]	=	Speed		 >>24;

		CAN_Transmit(CAN2, &TxMessage);	
}

void Can_9025_Send2(int16_t control,int32_t Speed)
{
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x142;																																												 					 // ��׼��ʶ�� 				   																																								 // ������չ��ʾ�� 
		TxMessage.IDE=CAN_Id_Standard;						 																																		 					 // ��׼֡
		TxMessage.RTR=CAN_RTR_Data;		 																																													 // ����֡
		TxMessage.DLC=0x08;		
		TxMessage.Data[0] =	control;
		
		TxMessage.Data[4]	=	Speed<<24>>24;
		TxMessage.Data[5]	=	Speed<<16>>24;
		TxMessage.Data[6]	=	Speed<< 8>>24;
		TxMessage.Data[7]	=	Speed		 >>24;

		CAN_Transmit(CAN2, &TxMessage);	
}

void Can_9025_stop1()
{
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x141;																																												 					 // ��׼��ʶ�� 				   																																								 // ������չ��ʾ�� 
		TxMessage.IDE=CAN_Id_Standard;						 																																		 					 // ��׼֡
		TxMessage.RTR=CAN_RTR_Data;		 																																													 // ����֡
		TxMessage.DLC=0x08;		
		TxMessage.Data[0] =	0x81;
		
		TxMessage.Data[4]	=	0;
		TxMessage.Data[5]	=	0;
		TxMessage.Data[6]	=	0;
		TxMessage.Data[7]	=	0;

		CAN_Transmit(CAN2, &TxMessage);	
}

void Can_9025_stop2()
{
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x142;																																												 					 // ��׼��ʶ�� 				   																																								 // ������չ��ʾ�� 
		TxMessage.IDE=CAN_Id_Standard;						 																																		 					 // ��׼֡
		TxMessage.RTR=CAN_RTR_Data;		 																																													 // ����֡
		TxMessage.DLC=0x08;		
		TxMessage.Data[0] =	0x81;
		
		TxMessage.Data[4]	=	0;
		TxMessage.Data[5]	=	0;
		TxMessage.Data[6]	=	0;
		TxMessage.Data[7]	=	0;

		CAN_Transmit(CAN2, &TxMessage);	
}


int RxMessage_i;

void CAN1_RX0_IRQHandler(void)
{
	//	IWDG_Feed();
    CanRxMsg RxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    RxMessage_i=RxMessage.StdId;
    if(RxMessage_i>=0x10&&RxMessage_i<=0x13)
    {				
//				if(RxMessage_i-0x10==0)a++;
//				if(RxMessage_i-0x10==1)b++;
//				if(RxMessage_i-0x10==2)c++;
//				if(RxMessage_i-0x10==3)d++;
				DM_get_motor_measure(DM_data[RxMessage_i-0x10],RxMessage);
		}	
}
    CanRxMsg RxMessage2;
void CAN2_RX1_IRQHandler(void)
{
		static int i,motor_ID;

    CAN_Receive(CAN2, CAN_FIFO1, &RxMessage2);
		i=RxMessage2.StdId;
		switch (i)
		{
			case 0x141:
			case 0x142:
			{
				motor_ID=RxMessage2.StdId-0x140;																																						///////////////////////////���ID����
				motor_ID=motor_ID-1;
				
				MF9025[motor_ID].Motor_Temp					= RxMessage2.Data[1];
				MF9025[motor_ID].Iq[0] 							= RxMessage2.Data[2];
				MF9025[motor_ID].Iq[1] 							= RxMessage2.Data[3];
				MF9025[motor_ID].SPEED[0] 					= RxMessage2.Data[4];
				MF9025[motor_ID].SPEED[1] 					= RxMessage2.Data[5];
				MF9025[motor_ID].Encoder[0] 				= RxMessage2.Data[6];
				MF9025[motor_ID].Encoder[1]					= RxMessage2.Data[7];
				MF9025[motor_ID].RealEncoder				=	(int16_t)MF9025[motor_ID].Encoder[0]|(int16_t)(MF9025[motor_ID].Encoder[1])<<8;
				MF9025[motor_ID].RealSpeed					=	(int16_t)MF9025[motor_ID].SPEED[0]|(int16_t)MF9025[motor_ID].SPEED[1]<<8;
				MF9025[motor_ID].RealIq							=	(int16_t)MF9025[motor_ID].Iq[0]|(int16_t)MF9025[motor_ID].Iq[1]<<8;
				
				if((MF9025[motor_ID].RealEncoder-MF9025[motor_ID].EncoderLast)>32767)
						MF9025[motor_ID].CircleNUM--;
				else if((MF9025[motor_ID].RealEncoder-MF9025[motor_ID].EncoderLast)<-32767)
						MF9025[motor_ID].CircleNUM++;
				
				MF9025[motor_ID].EncoderLast=MF9025[motor_ID].RealEncoder;
				
			}break;
			case 0x205: 		get_motor_measure(&yaw_data,RxMessage2);
											yaw_motor.relative_angle=(360.0f/8192.0f)*get_relative_pos(yaw_data.ecd,616.0f);
											break;
			case 0x400:			RC_Mode=RxMessage2.Data[0];break;
			case 0x401:
			{	
				
				rc_ctrl.rc.ch[2]	=		RxMessage2.Data[2]|RxMessage2.Data[3]<<8;
				rc_ctrl.rc.ch[3]	=		RxMessage2.Data[4]|RxMessage2.Data[5]<<8;
				rc_ctrl.rc.ch[4]	=		RxMessage2.Data[6]|RxMessage2.Data[7]<<8;
			}
			break;
			case 0x402:
			{	
				rc_ctrl.rc.ch[0]	=	 	RxMessage2.Data[0]|RxMessage2.Data[1]<<8;
				rc_ctrl.rc.ch[1]	=		RxMessage2.Data[2]|RxMessage2.Data[3]<<8;
				
			}
			break;			
    }
}








