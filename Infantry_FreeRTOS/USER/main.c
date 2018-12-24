#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "freertostask.h"   


int main()
{
	/*设置优先级分组4，即4位抢占优先级，0位相应优先级*/
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(180);		  //初始化滴答定时器
	
	/*创建开始任务并开启调度器*/
	FreeRTOS_init();

	/*Never arrive here*/
	while(1)
	{  
 	
   	}
} 



