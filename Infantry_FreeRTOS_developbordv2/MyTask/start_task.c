#include "start_task.h"
#include "FreeRTOS.h"  
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "MotorCan.h"
#include "RemotDbus.h"
#include "event_groups.h"
#include "detect_task.h"


/*创建队列句柄*/
 QueueHandle_t FrictionControlQueue;
 QueueHandle_t Can1ReceiveQueue;
 QueueHandle_t Can2ReceiveQueue;
 QueueHandle_t GimbalDataQueue;
 QueueHandle_t CMDataQueue;
 
  /*创建信号量句柄*/
xSemaphoreHandle DbusParseSem;
xSemaphoreHandle ImuDataSem;
//static  xSemaphoreHandle FrictionCaliTriggerSem;
//static  xSemaphoreHandle FrictionCaliSem;
//static  xSemaphoreHandle ImuGimbalCaliTriggerSem;
//static  xSemaphoreHandle ImuGimbalCaliSem;

/*创建事件标记组*/
EventGroupHandle_t BolletFreControlEnt;
EventGroupHandle_t CmGimbalOutputEnt;
EventGroupHandle_t OfflineDetectedEnt;

/*创建任务句柄*/
 TaskHandle_t DetectTask_Handler;
 TaskHandle_t StartTask_Handler;

void start_task(void *pvParameters)
{	
	taskENTER_CRITICAL();
	/*创建队列*/
	FrictionControlQueue = xQueueCreate(1,sizeof(int8_t));
	Can1ReceiveQueue     = xQueueCreate(1,sizeof(CanRxMsg));
	Can2ReceiveQueue     = xQueueCreate(1,sizeof(CanRxMsg));
	GimbalDataQueue      = xQueueCreate(1,sizeof(struct GimbalControlValue));
	CMDataQueue          = xQueueCreate(1,sizeof(struct CMControlValue));
	/*创建信号量*/
	DbusParseSem   =  xSemaphoreCreateBinary();
	ImuDataSem     =  xSemaphoreCreateBinary();
	
	/*创建事件标志组*/
	BolletFreControlEnt      = xEventGroupCreate();
	CmGimbalOutputEnt        = xEventGroupCreate();
	OfflineDetectedEnt       = xEventGroupCreate();
	
	/*创建任务*/
	xTaskCreate(DetectTask,          "Detect_Task"       ,128 ,NULL,2,&DetectTask_Handler);
//	xTaskCreate(DbusParseTask,       "dbustask"           ,1024,NULL,2,&DbusParseTask_Handler);	
//	xTaskCreate(FrictionControlTask, "friMoterControltask",512 ,NULL,3,&FrictionControlTask_Handler);
//    xTaskCreate(FrictionCaliTask,    "calitask"           ,128 ,NULL,4,&FrictionCaliTask_Handler);	
//	xTaskCreate(ImuGimbalCaliTask,   "imucalitask"        ,128  ,NULL,7,&ImuGimbalCaliTask_Handler);	
//	xTaskCreate(Can1ReceiveTask,     "Can1Receivetask"    ,300  ,NULL,5,&Can1ReceiveTask_Handler);	
//	xTaskCreate(Can2ReceiveTask,     "Can2Receivetask"    ,300  ,NULL,5,&Can2ReceiveTask_Handler);	
//	xTaskCreate(IMUReceiveTask,      "IMUReceivetask"     ,128  ,NULL,5,&IMUReceiveTask_Handler);
//    xTaskCreate(ChassisControlTask,  "CMControltask"      ,512  ,NULL,3,&ChassisControlTask_Handler);
	//xTaskCreate(OfflineDetectedTask, "Offlinedetecttask"  ,512  ,NULL,3,&OfflineDecetedTask_Handler);
	
	taskEXIT_CRITICAL();
	
	
//	char  * RunTimeInfo;
//	char *taskstate  = NULL;
	
	//RunTimeInfo   =  (char *)pvPortMalloc(sizeof(char) * 400);
	
//	taskstate = (char *)pvPortMalloc(sizeof(char) * 2048);
    for(;;)
	{	
//		memset(RunTimeInfo,0,400);
//		vTaskGetRunTimeStats(RunTimeInfo);
	
		
//		printf("任务名\t\t运行时间\t所占百分比\r\n");
//		printf("%s\r\n",RunTimeInfo);

		//		vTaskList(taskstate);
		//	printf("任务名\t\t任务状态\t任务优先级\t堆栈使用情况\t任务标签\r\n");
//		printf("%s\r\n",taskstate);
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_14);
		vTaskDelay(500);
		
	}
}

void StartTask()
{
    	xTaskCreate((TaskFunction_t)     start_task   , 
						 (const char *)  "start_task" ,
						 (uint16_t)      200          ,
						 (void *)         NULL         ,    
						 (UBaseType_t)    2             ,	
						 (TaskHandle_t *) &StartTask_Handler);
	  //开启任务调度器				 
	   vTaskStartScheduler();
}
