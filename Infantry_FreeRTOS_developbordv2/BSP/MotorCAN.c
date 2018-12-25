#include "MotorCAN.h"

static void MotorCAN1_Init()
{
	GPIO_InitTypeDef       gpio;  	
	NVIC_InitTypeDef       nvic;
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;		        									
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD, &gpio);
	
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 
	
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 5;    
	nvic.NVIC_IRQChannelSubPriority = 0;          
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);	
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 5;    
	nvic.NVIC_IRQChannelSubPriority = 0;           
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
  
	can.CAN_TTCM = DISABLE;		 
	can.CAN_ABOM = DISABLE;		 
	can.CAN_AWUM = DISABLE;			 
	can.CAN_NART = DISABLE;			 
	can.CAN_RFLM = DISABLE;			 
	can.CAN_TXFP = ENABLE;			 
	can.CAN_Mode = CAN_Mode_Normal;		 
	can.CAN_SJW  = CAN_SJW_1tq; 
	can.CAN_BS1 = CAN_BS2_6tq;
	can.CAN_BS2 = CAN_BS1_8tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+6+8)/3=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0; 
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; 
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;                    
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0; 
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);	
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		 
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		 
	
}

static void MotorCAN2_Init()
{
	GPIO_InitTypeDef       gpio;  	
	NVIC_InitTypeDef       nvic;
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;		        									
	gpio.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &gpio);
	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	
	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 4;    
	nvic.NVIC_IRQChannelSubPriority = 0;          
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);	
	
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 6;    
	nvic.NVIC_IRQChannelSubPriority = 0;           
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN2);
	CAN_StructInit(&can);
  
	can.CAN_TTCM = DISABLE;		 
	can.CAN_ABOM = DISABLE;		 
	can.CAN_AWUM = DISABLE;			 
	can.CAN_NART = DISABLE;			 
	can.CAN_RFLM = DISABLE;			 
	can.CAN_TXFP = ENABLE;			 
	can.CAN_Mode = CAN_Mode_Normal;		 
	can.CAN_SJW  = CAN_SJW_1tq; 
	can.CAN_BS1 = CAN_BS2_6tq;
	can.CAN_BS2 = CAN_BS1_8tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+6+8)/3=1Mbps
	CAN_Init(CAN2, &can);

	can_filter.CAN_FilterNumber=14; 
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; 
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;                    
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0; 
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);	
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		 
 	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE); 	
}


void MoterCanInit()
{
	MotorCAN1_Init();
	MotorCAN2_Init();
}

void CanSendMess(CAN_TypeDef* CANx,uint32_t SendID,int16_t *message)
{
	int i=0;
	u8 mbox;
	
    CanTxMsg tx_message;
    tx_message.StdId = SendID; 
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data; 
    tx_message.DLC = 0x08; 
	
    tx_message.Data[0] = (uint8_t)(message[0] >> 8); 
    tx_message.Data[1] = (uint8_t) message[0];          
    tx_message.Data[2] = (uint8_t)(message[1] >> 8);
    tx_message.Data[3] = (uint8_t) message[1];
    tx_message.Data[4] = (uint8_t)(message[2] >> 8);
    tx_message.Data[5] = (uint8_t) message[2];
    tx_message.Data[6] = (uint8_t)(message[3] >> 8);
    tx_message.Data[7] = (uint8_t) message[3];
	
    mbox = CAN_Transmit(CANx,&tx_message);
 while(CAN_TransmitStatus(CANx,mbox) ==CAN_TxStatus_Failed && i++<0xffff);	
	
}


	


