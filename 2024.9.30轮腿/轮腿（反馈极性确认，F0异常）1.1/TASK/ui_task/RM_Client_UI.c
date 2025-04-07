/*************************************************************
  ****************************(C) COPYRIGHT 2021 NCIST****************************
  * @file       UI_task.c/h
  * @brief      上位机通信任务，视觉数据处理
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     2021           zhuxunfu              finished
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */

#include "RM_Client_UI.h"

#define Get_CRC16_Check_Sum_UI get_crc16_check_sum
#define Get_CRC8_Check_Sum_UI get_crc8_check_sum 

//extern ext_game_robot_status_t	GameRobotStat;
//extern Super_power_t Super_power;
//extern int EE_Flag,FF_Flag;
//uint8_t VV_Flag=0;
//float superpower_x,super_ratio;
//String_Data CH_FRICTION[2],CH_CHASSIC[2],Power[2];
//Graph_Data Shoot_Mode[2];

//char chassis_follow_mode[14]= "CHASSIC:FOLLOW",chassis_dog_mode[14] = "CHASSIC: DODGE";
//char friction_off[3] = "OFF",friction_on[2] = "ON";
//char Super_cap[5]={0};
/****************************************************************************************/
unsigned char UI_Seq;                      //包序号
u32 Robot_ID,Cilent_ID;


void UI_Task(void *pvParameters)
{	
//	///先添加一个ID的UI然后用另一个结构体使用这个ID去刷新UI 目前添加四个都可以使用
//	Char_Draw(&CH_CHASSIC[0],	"002",UI_Graph_ADD,1,UI_Color_Pink,18,13,2,180,700,&chassis_follow_mode[0]);
//	Char_Draw(&CH_FRICTION[0],"001",UI_Graph_ADD,1,UI_Color_Pink,18,4,2,1400,620,&friction_off[0]);//摩擦轮状态
//	Char_Draw(&Power[0],			"005",UI_Graph_ADD,1,UI_Color_Cyan,40,5,2,1400,780,&Super_cap[0]);
//	
//	Char_ReFresh(CH_FRICTION[0]);	
//	Char_ReFresh(Power[0]);
//	Char_ReFresh(CH_CHASSIC[0]);
//for(;;)
// {
//	 
//	 
//		if(IF_KEY_PRESSED_V&&!V_Flag&&rc_ctrl.key.v)
//		{
//				V_Flag = 1;
//		}				
//		if(V_Flag==1&&!IF_KEY_PRESSED_V)
//		{
//				VV_Flag++;
//				VV_Flag%=2;
//				V_Flag=0;
//		}
//	 
//		if(VV_Flag)
//		{
//		
///************************************************绘制字符型数据*************************************************
//**参数：*image Graph_Data类型变量指针，用于存放图形数据
//        imagename[3]   图片名称，用于标识更改
//        Graph_Operate   图片操作，见头文件
//        Graph_Layer    图层0-9
//        Graph_Color    图形颜色
//        Graph_Width    图形线宽
//        Graph_Size     字号
//        Graph_Digit    字符个数
//        Start_x、Start_x    开始坐标
//        *Char_Data          待发送字符串开始地址
//**********************************************************************************************************/
//				

//	//Char_Draw(&CH_CHASSIC[5],"078",UI_Graph_ADD,1,UI_Color_Pink,18,13,2,180,620,&track_normal[0]);//云台状态（）
//			if(EE_Flag==0)
//			{
//				memset(&CH_FRICTION[1],0,sizeof(CH_FRICTION[1]));
//				Char_Draw(&CH_FRICTION[1],"001",UI_Graph_Change,1,UI_Color_Green,18,4,2,1400,620,&friction_off[0]);//摩擦轮状态
//				
//			}
//			else if(EE_Flag)
//			{
//				memset(&CH_FRICTION[1],0,sizeof(CH_FRICTION[1]));
//				Char_Draw(&CH_FRICTION[1],"001",UI_Graph_Change,1,UI_Color_Purplish_red,18,4,2,1400,620,&friction_on[0]);//摩擦轮状态	
//			}			
//			if(FF_Flag==0)
//			{
//				memset(&CH_CHASSIC[1],0,sizeof(CH_CHASSIC[1]));
//				Char_Draw(&CH_CHASSIC[1],"002",UI_Graph_Change,1,UI_Color_Green,18,14,2,180,700,&chassis_follow_mode[0]);
//			}
//			else if(FF_Flag)
//			{
//				memset(&CH_CHASSIC[1],0,sizeof(CH_CHASSIC[1]));
//				Char_Draw(&CH_CHASSIC[1],"002",UI_Graph_Change,1,UI_Color_Yellow,18,14,2,180,700,&chassis_dog_mode[0]);
//			}
//		
//			
//			
//			
///****************************************换弹***************************************************/			
//			
//	
//			
///****************************************超级电容***************************************************/

//		superpower_x = Super_power.volt;

//		if(superpower_x >= 22000.0f)
//		{
//				superpower_x = 22000.0f;
//		}
//		else if(superpower_x <= 12000.0f)
//		{
//				superpower_x = 12000.0f;
//		}
//		super_ratio = (superpower_x - 12000.0f) / 10000.0f * 100.0f;	
//		sprintf(Super_cap, "%0.3f", super_ratio);
//		memset(&Power[1],0,sizeof(Power[1]));
//		Char_Draw(&Power[1],"005",UI_Graph_Change,1,UI_Color_Cyan,40,5,2,1400,780,&Super_cap[0]);
///****************************************底盘位置***************************************************/
//	
//		if(GameRobotStat.robot_id == 3)
//			{
//				Robot_ID = 3;
//				Cilent_ID = 0x0103;
//			}

//			if(GameRobotStat.robot_id == 4)
//			{
//				Robot_ID = 4;
//				Cilent_ID = 0x0104;
//			}

//			if(GameRobotStat.robot_id == 5)
//			{
//				Robot_ID = 5;
//				Cilent_ID = 0x0105;
//			}

//			if(GameRobotStat.robot_id == 103)
//			{
//				Robot_ID = 103;
//				Cilent_ID = 0x0167;
//			}

//			if(GameRobotStat.robot_id == 104)
//			{
//				Robot_ID = 104;
//				Cilent_ID = 0x0168;
//			}

//			if(GameRobotStat.robot_id == 105)
//			{
//				Robot_ID = 105;
//				Cilent_ID = 0x0169;
//			}
//		Char_ReFresh(CH_FRICTION[0]);	
//		Char_ReFresh(CH_CHASSIC[0]);
//		Char_ReFresh(Power[0]);
//		
//		Char_ReFresh(CH_FRICTION[1]);	
//		Char_ReFresh(CH_CHASSIC[1]);
//		Char_ReFresh(Power[1]);
//		
//						
//	}
//		vTaskDelay(1);
//	}
}


/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
   USART_SendData(USART1,ch);
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   //UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
   image->graphic_name[i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->Graph_Control.graphic_name[i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      }                                               //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   //UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   //UI_Seq++;                                                         //包序号+1
   return 0;
}



