#include "stm32f4xx.h"
#include "BasicPeripherals.h"
#include "FreeRTOS.h"
#include "task.h"

static void LEDConfig(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		 //PE7                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		//PF14  
	GPIO_Init(GPIOF,&GPIO_InitStructure);  
	
	GPIO_SetBits(GPIOF,GPIO_Pin_14);
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
}

static void Beep_PWM_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 90-1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 3814;  
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;	

	TIM_OC1Init(TIM3,&TIM_OCInitStructure);//PB4
	
	
	TIM_Cmd(TIM3,ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
}

static void Key_init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
}

static void LaserConfig(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		//PG13                                                                                                                                                                                                                                                                                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
}

void TIM4_Config(void);

void BasicPreiph_Init()
{
	LEDConfig();
//	Beep_PWM_Config();
	Key_init();
	LaserConfig();
	
}

void StrartingMusic()
{
	    TIM3->CCR1 = 3000;
		 TIM3->ARR = BEEP_NOTE_1;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_2;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_3;
		 vTaskDelay(150);
		 TIM3->CCR1 = 0;
}

void FrictionCaliTriggerMusic()
{
	    TIM3->CCR1 = 3000;
		 TIM3->ARR = BEEP_NOTE_3;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_2;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_1;
		 vTaskDelay(150);
		 TIM3->CCR1 = 0;
}

void FrictionCaliMusic()
{
	    TIM3->CCR1 = 3000;
		 TIM3->ARR = BEEP_NOTE_1;
		vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_2;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_3;
		 vTaskDelay(150);
		 TIM3->CCR1 = 0;
}

void ImuCaliMusic()
{
	    TIM3->CCR1 = 3000;
		 TIM3->ARR = BEEP_NOTE_1;
		vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_1;
		 vTaskDelay(150);
		
		 TIM3->ARR = BEEP_NOTE_2;
		 vTaskDelay(150);
		 TIM3->CCR1 = 0;
}

void TIM4_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef   NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    tim.TIM_Period = 50-1;
    tim.TIM_Prescaler = 90 - 1;	           
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	 
    tim.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_ARRPreloadConfig(TIM4, ENABLE);	 
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_Cmd(TIM4,ENABLE);
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
	
 
	
}

volatile unsigned long FreeRTOSRunTimeTicks ;
void TIM4_IRQHandler(void)
{
 if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		FreeRTOSRunTimeTicks++; 
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);   
}


void ConfigureTimeForTimeStats()
{
   
	FreeRTOSRunTimeTicks = 0;
	TIM4_Config();//统计任务时间
}







