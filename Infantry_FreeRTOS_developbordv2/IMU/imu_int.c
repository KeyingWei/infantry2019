#include "stm32f4xx.h"
#include "imu.h"
#include "imu_int.h"
#include "MPU6500_driver.h"
/**********************************
��������ImuInt_Gpio_Config
��ڲ�������
���ڲ�������
���ܣ�����PE1��MPU_INT����
*********************************/
static void ImuInt_Gpio_Config()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);// GPIOEʱ��
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // MPU_INT����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOE1

}
/**********************************
��������IMU_NVIC_Config
��ڲ�������
���ڲ�������
���ܣ������ⲿ�ж�1�ж����ȼ�
*********************************/
static void IMU_NVIC_Config()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�1
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x6;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}
/**********************************
��������IMU_NVIC_Config
��ڲ�������
���ڲ�������
���ܣ������ⲿ�ж�1������ʽ
*********************************/
static void IMU_EXTI_Config()
{
  EXTI_InitTypeDef   EXTI_InitStructure;
	
	/* ����EXTI_Line1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);//�����ж���1
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStructure);//����
}
/*****************************************
��������IMU_NVIC_Config
��ڲ�������
���ڲ�������
���ܣ������ⲿ�ж�1�����ţ�NVIC���ʹ�����ʽ
******************************************/
void IMU_Config()
{
	MPU6500Config();
    ImuInt_Gpio_Config();
	IMU_NVIC_Config();
	IMU_EXTI_Config();
    
}



