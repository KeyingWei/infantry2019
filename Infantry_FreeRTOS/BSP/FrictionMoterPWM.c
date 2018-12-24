#include "FrictionMoterPWM.h"
#include "stm32f4xx.h"

void FrictionMoterPWM_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	


	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6, GPIO_AF_TIM12);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource9, GPIO_AF_TIM12); 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;           //GPIOH6.H9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOH,&GPIO_InitStructure);            


	TIM_TimeBaseStructure.TIM_Prescaler = 900-1;  //0.1MHZ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period = 2000-1;  //50HZ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //PH6
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);  //PH9


	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);  


	TIM_ARRPreloadConfig(TIM12,ENABLE);

	TIM_Cmd(TIM12, ENABLE);  	
}


