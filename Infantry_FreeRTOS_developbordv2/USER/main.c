#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "freertostask.h"   


int main()
{
	/*�������ȼ�����4����4λ��ռ���ȼ���0λ��Ӧ���ȼ�*/
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(180);		  //��ʼ���δ�ʱ��
	
	/*������ʼ���񲢿���������*/
	FreeRTOS_init();

	/*Never arrive here*/
	while(1)
	{  
 	
   	}
} 



