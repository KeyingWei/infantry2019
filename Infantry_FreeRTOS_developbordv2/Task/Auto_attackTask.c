#include "Auto_attackTask.h"
#include <string.h>
#include "freertostask.h"

AUTODATA AutoData; 
volatile  u8 mainfocus_rx_buffer[10];
AUTODATA_From_MainFocus Auto_Origion ={0}; 


char  SendBuff[4];
void send_data(char mode, u8 cmd2)
{
	static unsigned char i = 0;
	
	SendBuff[0] = 0xFF;
	SendBuff[1] = mode;
	SendBuff[2] = cmd2;
	SendBuff[3] = 0xFE;
	
	for(i = 0; i < 4 ;i++)//循环发送数据
	{	 
		 while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);  //等待上次传输完成
		 USART_SendData(USART3,(uint8_t)SendBuff[i]);		 
	}
}

void receiveMainFocusData()
{
    int i = 0;
		
	  for(i =0;i< 10;i++)
	  {
		  if(mainfocus_rx_buffer[i]== 0xFF && mainfocus_rx_buffer[i+7]== 0xFE)
			{
			  memcpy(&Auto_Origion,(u8 *)&mainfocus_rx_buffer[i+1],6);
			}
	 }	
		AutoData.YawAxiaAngle  = -(float)Auto_Origion.YawAxiaAngle /100 ;
		AutoData.PitchAxiaAngle = -(float)Auto_Origion.PitchAxiaAngle /100;
		AutoData.Mode = Auto_Origion.Mode;	
		
		
   if(AutoData.Mode == FPS_IS_120)
	{	
		AutoData.PitchAxiaAngle = AutoData.PitchAxiaAngle ;// 
	}
	if(AutoData.Mode == FPS_IS_60)
	{
		AutoData.PitchAxiaAngle = AutoData.PitchAxiaAngle ;// 
	}
	

}	




