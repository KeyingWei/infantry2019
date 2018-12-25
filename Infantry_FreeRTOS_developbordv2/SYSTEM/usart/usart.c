#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOSConfig.h"					
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	
#if ( USART_FOR_PRINT == 2)	
	USART2->DR = (u8) ch;
    while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
#elif ( USART_FOR_PRINT == 3)
	USART3->DR = (u8) ch;
	while((USART3->SR&0X40)==0);//ѭ������,ֱ��������� 
#else
	USART6->DR = (u8) ch;
	while((USART6->SR&0X40)==0);//ѭ������,ֱ��������� 
#endif
	return ch;
}
#endif
 
//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{
   //GPIO�˿�����
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
#if EN_USART_RX
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	
#if ( USART_FOR_PRINT == 2)	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5����ΪUSART2TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6����ΪUSART2RX

#elif ( USART_FOR_PRINT == 3)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��	

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD5����ΪUSART3TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD6����ΪUSART3RX
#else 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOGʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART6,ENABLE);//ʹ��USART6ʱ��	

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART6); //GPIOD5����Ϊ USART6RX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_USART6); //GPIOD6����ΪUSART6TX
#endif

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
#if ( USART_FOR_PRINT == 2)	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;  
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
 #elif ( USART_FOR_PRINT == 3)  
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;  
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
#else
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;  
	GPIO_Init(GPIOG,&GPIO_InitStructure); 
#endif


   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
#if ( USART_FOR_PRINT == 2)	
  USART_Init(USART2, &USART_InitStructure);  
   USART_Cmd(USART2, ENABLE);  
#elif ( USART_FOR_PRINT == 3)  
  USART_Init(USART3, &USART_InitStructure);  
   USART_Cmd(USART3, ENABLE);  
#else
  USART_Init(USART6, &USART_InitStructure); 
  USART_Cmd(USART6, ENABLE); 	  
#endif	

#if EN_USART1_RX	
		#if ( USART_FOR_PRINT == 2)	
		   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
		   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
		#elif ( USART_FOR_PRINT == 3)  
		   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�
		   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
		#else
		   USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�
		   NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����6�ж�ͨ�� 
		#endif	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
#endif
}


#if EN_USART_RX   //���ʹ���˽���
//�����жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

void USART2_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		 
    } 

} 
#endif	

 



