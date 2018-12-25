#include "tim4.h"
extern "C"
{
	#include "bsp.h"
}

void TIM4_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = 19; 
	TIM_TimeBaseStructure.TIM_Prescaler = 8399;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4, ENABLE);  
}

void TIME4NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
}

void TIM4Config(void)
{
	TIM4_Config();
	TIME4NVIC_Config();
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) //����ж�
	{
			printf("asdasd\r\n");
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //����жϱ�־λ
}

