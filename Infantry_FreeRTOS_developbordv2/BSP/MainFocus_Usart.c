#include "MainFocus_Usart.h"
#include "Auto_attackTask.h"
/****************************
��������USART3_Gpio_Config
��ڲ�������
���ڲ�������
���ܣ���ʼ������3GPIO 
******************************/
void USART3_Gpio_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;	                                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9 ,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8 ,GPIO_AF_USART3);
}	
/****************************
��������USART1_Config
��ڲ�������
���ڲ�������
���ܣ���ʼ������1����
˵���������ʣ�100K
			8λ����λ
			1��ֹͣλ
      żУ��
******************************/
void USART3_Config(void)
{
	USART_InitTypeDef USART_InitStructure;	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 115200;	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure);
	
	USART_Cmd(USART3,ENABLE);											
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);	
}
/****************************
��������DMA2NVIC_Config
��ڲ�������
���ڲ�������
���ܣ�����DMA2������5��ӦUSART1_RX�ж�
      ��ռ���ȼ���0
			��Ӧ���ȼ���3
******************************/
void DMA1NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/****************************
��������DMA2_Config
��ڲ�������
���ڲ�������
���ܣ�����DMA2������5��ͨ���ģ�USART1_TX��
˵�������ݳ��ȣ�8
			�������赽�ڴ�  
******************************/
void DMA1_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);
	
	DMA_Cmd(DMA1_Stream1, DISABLE);						
  while (DMA1_Stream1->CR & DMA_SxCR_EN);
	
	DMA_DeInit(DMA1_Stream1);	
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);	
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mainfocus_rx_buffer;			
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									
	DMA_InitStructure.DMA_BufferSize = 8;																
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;							
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1,&DMA_InitStructure);

	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
  DMA_Cmd(DMA1_Stream1,ENABLE);
}
/****************************
��������DbusConfig
��ڲ�������
���ڲ�������
���ܣ����ô���3������������
******************************/
void MainFocusConfig(void)
{
	USART3_Gpio_Config();
	USART3_Config(); 
	DMA1NVIC_Config();
	DMA1_Config();
}
