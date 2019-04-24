#include "start_task.h"
#include "FreeRTOS.h"  
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "MotorCan.h"
#include "RemotDbus.h"
#include "event_groups.h"
#include "detect_task.h"


/*�������о��*/
 QueueHandle_t FrictionControlQueue;
 QueueHandle_t Can1ReceiveQueue;
 QueueHandle_t Can2ReceiveQueue;
 QueueHandle_t GimbalDataQueue;
 QueueHandle_t CMDataQueue;
 
  /*�����ź������*/
xSemaphoreHandle DbusParseSem;
xSemaphoreHandle ImuDataSem;
//static  xSemaphoreHandle FrictionCaliTriggerSem;
//static  xSemaphoreHandle FrictionCaliSem;
//static  xSemaphoreHandle ImuGimbalCaliTriggerSem;
//static  xSemaphoreHandle ImuGimbalCaliSem;

/*�����¼������*/
EventGroupHandle_t BolletFreControlEnt;
EventGroupHandle_t CmGimbalOutputEnt;
EventGroupHandle_t OfflineDetectedEnt;

/*����������*/
 TaskHandle_t DetectTask_Handler;
 TaskHandle_t StartTask_Handler;

void start_task(void *pvParameters)
{	
	taskENTER_CRITICAL();
	/*��������*/
	FrictionControlQueue = xQueueCreate(1,sizeof(int8_t));
	Can1ReceiveQueue     = xQueueCreate(1,sizeof(CanRxMsg));
	Can2ReceiveQueue     = xQueueCreate(1,sizeof(CanRxMsg));
	GimbalDataQueue      = xQueueCreate(1,sizeof(struct GimbalControlValue));
	CMDataQueue          = xQueueCreate(1,sizeof(struct CMControlValue));
	/*�����ź���*/
	DbusParseSem   =  xSemaphoreCreateBinary();
	ImuDataSem     =  xSemaphoreCreateBinary();
	
	/*�����¼���־��*/
	BolletFreControlEnt      = xEventGroupCreate();
	CmGimbalOutputEnt        = xEventGroupCreate();
	OfflineDetectedEnt       = xEventGroupCreate();
	
	/*��������*/
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
	
		
//		printf("������\t\t����ʱ��\t��ռ�ٷֱ�\r\n");
//		printf("%s\r\n",RunTimeInfo);

		//		vTaskList(taskstate);
		//	printf("������\t\t����״̬\t�������ȼ�\t��ջʹ�����\t�����ǩ\r\n");
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
	  //�������������				 
	   vTaskStartScheduler();
}
