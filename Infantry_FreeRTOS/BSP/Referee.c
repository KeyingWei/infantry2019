#include "Referee.h"
#include <string.h>
/** 
* @brief: read the data of referee system
* @write by: WKY
*/
static u8 Rx_Buf_Referee[AHRS_RC_LEN]={0};		//定义接收缓冲 
u16 Usart1_RX_Cou = 0;
u8 Usart1_RX_Flag = 0;

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
static  void Referee_read_date(void); 

void Referee_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	//使能USART1，GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	//USART1_TX   GPIOA.9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 ; //GPIOA9与GPIOA10
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStruct); //初始化PA9，PA10
	

  DMA_DeInit(DMA2_Stream2);   //将DMA的通道1寄存器重设为缺省值
	
  DMA_Cmd(DMA2_Stream2, DISABLE);						
  while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Referee;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = AHRS_RC_LEN;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);//初始化DMA Stream
	
//	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
  DMA_Cmd(DMA2_Stream2,ENABLE);


   //USART 初始化设置
    USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx ;	//收发模式

    USART_Init(USART6, &USART_InitStructure); //初始化串口1
                    //使能串口1 
	USART_Cmd(USART6, ENABLE);  
    USART_ITConfig(USART6,USART_IT_TC,DISABLE);       
    USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);    
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	  
	 
	 NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;                
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         
     NVIC_Init(&NVIC_InitStructure);        

	 USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);	
}
 
 
void USART6_IRQHandler(void)  
{  
    u16 i=0 ;
     i = i;	
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) 
    {  
        i = USART6->SR;   //clear RXNE flag 
        i = USART6->DR;  
			
		
        DMA_Cmd(DMA2_Stream2, DISABLE); 	
          
        Referee_read_date();	//read data
			
        DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1); //clear DMA flag
        DMA_SetCurrDataCounter(DMA2_Stream2, AHRS_RC_LEN);  			
        DMA_Cmd(DMA2_Stream2, ENABLE); 
        		
    }  
		   __nop(); 
}  


void bubble_sort(float *p,char num)
{
  char i,j;
	float temp;
	for(i=0;i< num-1;i++)
	  for(j=0;j<num-1-i;j++)
	  { 
	     if(p[j]>p[j+1])
			 {
			      temp = p[j];
						p[j] = p[j+1];
				    p[j+1] = temp;
			 }
	  }
}



Referee_Date Referee_date1;
char  g_Referee_flag = 0 ;
Power power  ; 

static void Referee_read_date(void)
{
	u32 i;
	char frame_len;
	
  static float power_data[5];
	static char count = 0;
  float sum = 0;
	
	frame_len =  AHRS_RC_LEN - DMA_GetCurrDataCounter(DMA1_Stream5); //get the length of the frame
	//printf(" frame_len=%d\r\n",frame_len);
		for(i = 0; i < frame_len; i ++)
		{
			if(Rx_Buf_Referee[i] == 0xA5)
			{
				
				if(Verify_CRC8_Check_Sum(Rx_Buf_Referee,i + 5))
				{

					Referee_date1.frame_header.data_length = (Rx_Buf_Referee[i + 2] << 8) | Rx_Buf_Referee[i+1];
					Referee_date1.frame_header.seq =  Rx_Buf_Referee[i+3];
				
					
					Referee_date1.CmdID            =  Rx_Buf_Referee[i+5] | Rx_Buf_Referee[i+6] << 8;
				 
					if(Referee_date1.CmdID == ROBOT_STATE)
					{
						
						memcpy(&Referee_date1.GameRobotState_t,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
					
			//			printf("CmdID=%d  stageRemianTime=%d  gameProgress=%d  robotLevel=%d  remainHP=%d   maxHP=%d \r\n",Referee_date1.CmdID,Referee_date1.GameRobotState_t.stageRemianTime,Referee_date1.GameRobotState_t.gameProgress,\
						                                                                                                   Referee_date1.GameRobotState_t.robotLevel,Referee_date1.GameRobotState_t.remainHP,Referee_date1.GameRobotState_t.maxHP);
					}  
					else if(Referee_date1.CmdID == HURTDATA) 
					{							
					   Referee_date1.RoboHurt_t.armorType =  Rx_Buf_Referee[i+7]&0x0f;
						 Referee_date1.RoboHurt_t.hurtType  =  Rx_Buf_Referee[i+7]&0xf0;
						
				//		 printf("CmdID=%d  armorType= %d   hurtType=%d\r\n", Referee_date1.CmdID,Referee_date1.RoboHurt_t.armorType,Referee_date1.RoboHurt_t.hurtType);
						
					}
				 else if(Referee_date1.CmdID == SHOOTDATA)
					{
						memcpy(&Referee_date1.ShootData,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
	
					//	printf("CmdID=%d	bulletType=%d  bulletFreq=%d  bulletSpeed=%f\r\n",Referee_date1.CmdID,Referee_date1.ShootData.bulletType,Referee_date1.ShootData.bulletFreq,Referee_date1.ShootData.bulletSpeed);
					}
					
					else if(Referee_date1.CmdID == POWER_HEAT)
					{
						memcpy(&Referee_date1.powerHeatData,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
			//			printf("CmdID=%d	chassisVolt=%f  chassisCurrent=%f  chassisPower=%f  chassisPowerBuffer=%f  shooterHeat0=%d   shooterHeat1=%d\r\n ",\
										Referee_date1.CmdID,Referee_date1.powerHeatData.chassisVolt,Referee_date1.powerHeatData.chassisCurrent,Referee_date1.powerHeatData.chassisPower , Referee_date1.powerHeatData.chassisPowerBuffer,\
              			Referee_date1.powerHeatData.shooterHeat0,Referee_date1.powerHeatData.shooterHeat1);
					}
					else if(Referee_date1.CmdID == RFIDDETECT)
					{
						memcpy(&Referee_date1.RfidDetect,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
					//	printf("CmdID=%d	card_Type=%d	cardidx=%d\r\n",Referee_date1.CmdID ,Referee_date1.RfidDetect.card_Type,Referee_date1.RfidDetect.cardidx);
					}
					else if(Referee_date1.CmdID == GAMERUSULT)
					{
					  // Referee_date1.GameResult.winner = Rx_Buf_Referee[i + 7];
						 memcpy(&Referee_date1.GameResult,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
					//	printf("CmdID=%d  winner=%d\r\n ",Referee_date1.CmdID,Referee_date1.GameResult.winner);
					}
					else if(Referee_date1.CmdID == GETBUFF )
					{
						memcpy(&Referee_date1.GetBuff,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
					//	printf("CmdID=%d buffType=%d   buffAddition=%d",Referee_date1.CmdID,Referee_date1.GetBuff.buffType,Referee_date1.GetBuff.buffAddition);
				  }
					else if(Referee_date1.CmdID == POSITION)
					{		
						memcpy(&Referee_date1.GameRobotPos,&Rx_Buf_Referee[i+7],Referee_date1.frame_header.data_length);
			//			printf("CmID=%d  X=%f  Y=%f  Z=%f\r\n",Referee_date1.CmdID,Referee_date1.GameRobotPos.x,Referee_date1.GameRobotPos.y,Referee_date1.GameRobotPos.yaw);
					}
			  }
			 }
		}
		
		//对功率100ms采集一次
		power.curr_power =    Referee_date1.powerHeatData.chassisPower;
	    power_data[count] =   Referee_date1.powerHeatData.chassisPower;
		 count++;
		 if(count == 5 )
		 {
		    count =0;
			  g_Referee_flag = 1 ;
			  bubble_sort(power_data,5);
			 // printf("%f %f %f %f %f\r\n",power_data[0],power_data[1],power_data[2],power_data[3],power_data[4]);
			  for(i=1;i<=3;i++)
			 {
			   sum += power_data[i];
			 }
			  power.average = sum / 3;
			  sum = 0;
	 // 		printf("power.average=%f\r\n",power.average);
			
		 } 
}

const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned
char ucCRC8)
{
		unsigned char ucIndex;
		while (dwLength--)
		{
				ucIndex = ucCRC8^(*pchMessage++);
				ucCRC8 = CRC8_TAB[ucIndex];
		}
		return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
		unsigned char ucExpected = 0;
		if ((pchMessage == 0) || (dwLength <= 2)) return 0;
		ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
		return ( ucExpected == pchMessage[dwLength-1] );
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}
uint16_t CRC_INIT = 0xffff;
