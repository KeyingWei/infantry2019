#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "freertostask.h"
#include "BasicPeripherals.h"
#include "FrictionMoterPWM.h"
#include "RemotDbus.h"
#include "usart.h"
#include "MotorCAN.h"
#include "RemotDbus.h"
#include "imu_int.h"
#include "imu.h" 
#include "pid_modify.h"
#include "stdlib.h"
#include "MPU6500_driver.h"
#include "stmflash.h"
#include "string.h"

#include "math.h"
#include "InfantryConfig.h"
#include "delay.h"
#include "Auto_attackTask.h"
#include "MainFocus_Usart.h"
#include "Referee.h"

/*��������*/
u8 FrictionCalistop  = 0;
u8 ImuGimbalCalistop = 0;
Motor_SpeedLoopData_t CM_Motor[4];
Motor_SpeedLoopData_t FireMotor;
Motor_Posi_A_Speed_LoopData_t YawMotor;
Motor_Posi_A_Speed_LoopData_t PitchMotor;
IMUDATA  Mpu6500Data;
struct RemotDataPack RemotData;

/*����������*/
 TaskHandle_t DbusParseTask_Handler;
 TaskHandle_t StartTask_Handler;
 TaskHandle_t FrictionCaliTask_Handler;
 TaskHandle_t FrictionControlTask_Handler;
 TaskHandle_t Can1ReceiveTask_Handler;
 TaskHandle_t Can2ReceiveTask_Handler;
 TaskHandle_t IMUReceiveTask_Handler;
 TaskHandle_t BSPInitTaskTask_Handler;
 TaskHandle_t PIDCalculateTask_Handler;
 TaskHandle_t ChassisControlTask_Handler;
 TaskHandle_t GimbalControlTask_Handler;
 TaskHandle_t ImuGimbalCaliTask_Handler;
 TaskHandle_t OfflineDecetedTask_Handler;
 
 /*�����ź������*/
xSemaphoreHandle DbusParseSem;
xSemaphoreHandle ImuDataSem;
static  xSemaphoreHandle FrictionCaliTriggerSem;
static  xSemaphoreHandle FrictionCaliSem;
static  xSemaphoreHandle ImuGimbalCaliTriggerSem;
static  xSemaphoreHandle ImuGimbalCaliSem;


/*�����¼������*/
EventGroupHandle_t BolletFreControlEnt;
EventGroupHandle_t CmGimbalOutputEnt;
EventGroupHandle_t OfflineDetectedEnt;

/*�������о��*/
 QueueHandle_t FrictionControlQueue;
 QueueHandle_t Can1ReceiveQueue;
 QueueHandle_t Can2ReceiveQueue;
 QueueHandle_t GimbalDataQueue;
 QueueHandle_t CMDataQueue;
/*����ԭ������*/
static void DbusParseTask(void *pvParameters);
static void start_task(void *pvParameters);
static void FrictionCaliTask(void *pvParameters);
static void FrictionControlTask(void *pvParameters);
static void Can1ReceiveTask(void *pvParameters);
static void Can2ReceiveTask(void *pvParameters);
static void IMUReceiveTask(void *pvParameters);
static void BSPInitTask(void *pvParameters);
static void PIDCalculateTask(void *pvParameters);
static void ChassisControlTask(void *pvParameters);
static void GimbalControlTask(void *pvParameters);
static void  ImuGimbalCaliTask(void *pvParameters);
static void OfflineDetectedTask(void *pvParameters);
static void Set_Fire_Fre(int16_t fre);

void FreeRTOS_init()
{
  	xTaskCreate((TaskFunction_t)   start_task  , 
						 (const char *)       "start_task",
						 (uint16_t)             200            ,
						(void *)                  NULL         ,    
						(UBaseType_t)       2               ,	
						 (TaskHandle_t *) &StartTask_Handler);
						
	vTaskStartScheduler();
}



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
	xTaskCreate(BSPInitTask,          "bspinittask"       ,128 ,NULL,4,&BSPInitTaskTask_Handler);
	xTaskCreate(DbusParseTask,       "dbustask"           ,1024,NULL,2,&DbusParseTask_Handler);	
	xTaskCreate(FrictionControlTask, "friMoterControltask",512 ,NULL,3,&FrictionControlTask_Handler);
    xTaskCreate(FrictionCaliTask,    "calitask"           ,128 ,NULL,4,&FrictionCaliTask_Handler);	
	xTaskCreate(ImuGimbalCaliTask,   "imucalitask"        ,128  ,NULL,7,&ImuGimbalCaliTask_Handler);	
	xTaskCreate(Can1ReceiveTask,     "Can1Receivetask"    ,300  ,NULL,5,&Can1ReceiveTask_Handler);	
	xTaskCreate(Can2ReceiveTask,     "Can2Receivetask"    ,300  ,NULL,5,&Can2ReceiveTask_Handler);	
	xTaskCreate(IMUReceiveTask,      "IMUReceivetask"     ,128  ,NULL,5,&IMUReceiveTask_Handler);
    xTaskCreate(ChassisControlTask,  "CMControltask"      ,512  ,NULL,3,&ChassisControlTask_Handler);
	xTaskCreate(OfflineDetectedTask, "Offlinedetecttask"  ,512  ,NULL,3,&OfflineDecetedTask_Handler);
	
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
		
		//send_data(0,1);
		GPIO_ToggleBits(GPIOF,GPIO_Pin_14);
		vTaskDelay(500);
		
	}
}

void BSPInitTask(void *pvParameters)
{
	int32_t calibuff[5];
	
	BasicPreiph_Init(); //��ʼ��led,beep,key,laser
    FrictionMoterPWM_Init(); //��ʼ��Ħ���ֿ���pwm
	RemotDbus_Init();//��ʼ��ң��DBUS
	uart_init(115200);//��ʼ����ӡ����
	MoterCanInit();//��ʼ��can1,can2
	MainFocusConfig();//��ʼ��������������ͨѶ
	Referee_init();//��ʼ���������ڲ���ϵͳͨѶ
	
	 #if(IMU_MODE == 1)
	  IMU_Config();//��ʼ��MPU6500
     #endif	
	  memset(calibuff,0,sizeof(calibuff));
	  STMFLASH_Read(ADDR_FLASH_SECTOR_7,calibuff,sizeof(calibuff)/sizeof(calibuff[0]));
	
	  YawMotor.position.encoderInitValue          = calibuff[0] ; 
	  PitchMotor.position.encoderInitValue        = calibuff[1] ;
	  Mpu6500Data.Gx_offset                       = calibuff[2] ;
	  Mpu6500Data.Gz_offset                       = calibuff[3] ;
	  Mpu6500Data.Gy_offset                       = calibuff[4] ;
	
	vTaskDelay(100);
    char start_cnt = 0; 	
    for(;;)
    {
		    if(RemotData.rc.ch1 == 1024) //ң�س�ʼ���ɹ�
			{
				 vTaskDelay(100); 
				 start_cnt++;
				 if(start_cnt >= 40)//�ȴ�4s��IMU�����ȶ�
				 {
					 start_cnt = 0;
					 StrartingMusic();
					 xTaskCreate(GimbalControlTask,"gimbalControltask",512,NULL,5,&GimbalControlTask_Handler);
					 xTaskCreate(PIDCalculateTask, "PIDCalculateTask", 128,NULL,6,&PIDCalculateTask_Handler);
					 vTaskDelete(NULL);					 
				 }
		
			}		
            else
               RemotDbus_Init();//���³�ʼ��ң��DBUS 

          vTaskDelay(50); 				
	}		
}
 struct  ControlValue control;	 
static void DbusParseTask(void *pvParameters)
{
   u8 limitFlag = 0;
   	EventBits_t ControlBits;
    for(;;)
	{
		/*�ȴ�dbus����*/
	   xSemaphoreTake(DbusParseSem,portMAX_DELAY);
		/*����Dbus����*/
		RemotData.RC_onlineCnt = 0;
		xEventGroupClearBits(OfflineDetectedEnt,(1<<1));
	
		   RemotData.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;
		   RemotData.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; 		//!< Channel 1
		   RemotData.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
		   RemotData.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
		   RemotData.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;															//!< Switch left
		   RemotData.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); 																	//!< Switch right	
		   RemotData.mousekey.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); 											//!< Mouse X axis
		   RemotData.mousekey.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);										  //!< Mouse Y axis
		   RemotData.mousekey.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); 										//!< Mouse Z axis
		   RemotData.mousekey.press_l = sbus_rx_buffer[12]; 																					//!< Mouse Left Is Press ?
		   RemotData.mousekey.press_r = sbus_rx_buffer[13]; 																					//!< Mouse Right Is Press ?
		   RemotData.mousekey.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); 	

		
		
		 
		  switch(RemotData.rc.s2)
		  {
			  case RC_SW_DOWN:
				  {
					  if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_W)
							control.CMControl.forwardOrbackward =   FORWARESPEED;    
					 else if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_S)
							control.CMControl.forwardOrbackward = - FORWARESPEED;   
					 else if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_A)
							control.CMControl.horizoltal =   - HORIZONTALSPEED; 
					 else if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_D)
							control.CMControl.horizoltal =    HORIZONTALSPEED; 
					 else
					 {
						control.CMControl.horizoltal        =  0; 
						control.CMControl.forwardOrbackward =  0;
					 }
						 
					 if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_Q)
						  {
						      control.CMControl.DnfanceAngle = CMSWINGANGLE;
							  control.CMControl.DnfanceSpeed = CMSWINGSPEED;
						  }     
					  else if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_E)
						  {
						     control.CMControl.DnfanceAngle = 0;
                             control.CMControl.DnfanceSpeed = 0; 							  
						  }
						  
						  
					 if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_SHIFT)
						  {
						       xEventGroupSetBits(CmGimbalOutputEnt,0x10); //�����Զ�ģʽ
							   
						  }     
					  else if(RemotData.mousekey.v == KEY_PRESSED_OFFSET_CTRL)
						  {
						      xEventGroupClearBits(CmGimbalOutputEnt,0x10); //�ر��Զ�ģʽ 							  
						  }					

				      ControlBits = xEventGroupGetBits(CmGimbalOutputEnt);
					  
					  if(RemotData.mousekey.press_l == 1)
						   xEventGroupSetBits(BolletFreControlEnt,0x01); //����������
					  else
						   xEventGroupClearBits(BolletFreControlEnt,0x01); //�رղ�����
					  
					  xEventGroupSetBits(BolletFreControlEnt,0x02);//Ħ���ֿ���
					  xEventGroupSetBits(CmGimbalOutputEnt,0x04); //����������
					  
					  
					 if((ControlBits & 0x10) == 0) //�ֶ�ģʽ
					 {
						control.GimbalControl.left_right -= RemotData.mousekey.x * MOUSE_LEFTRIGHT_SPEED;
						control.GimbalControl.updown     += RemotData.mousekey.y * MOUSE_UPDOWN_SPEED; 					 
					 }
          				  
				  }
				  break;
		      case RC_SW_MID:
				  {
					  control.CMControl.horizoltal        = (RemotData.rc.ch0 - 1024) * REMOT_HORIONTALSPEED;
					  control.CMControl.forwardOrbackward = (RemotData.rc.ch1 - 1024) * REMOT_FORWARDSPEED;
					  control.CMControl.DnfanceAngle = 0;					  

					   if((ControlBits & 0x10) == 0) 
					   {
					      control.GimbalControl.left_right -= (RemotData.rc.ch2 -1024) * REMOT_LEFTRIGHT_SPEED;
						  control.GimbalControl.updown     -= ((float)(RemotData.rc.ch3 -1024)) * REMOT_UPDOWN_SPEED; 	
					   }
					  
					  if(RemotData.rc.s1 == RC_SW_UP)
						     xEventGroupClearBits(BolletFreControlEnt,0x03);
					  else if(RemotData.rc.s1 == RC_SW_MID)
						  {
						     xEventGroupSetBits(BolletFreControlEnt,  0x02);
							 xEventGroupClearBits(BolletFreControlEnt,0x01);
						  }				  
					  else if(RemotData.rc.s1 == RC_SW_DOWN)
						    xEventGroupSetBits(BolletFreControlEnt,0x03);
					  
					  xEventGroupSetBits(CmGimbalOutputEnt,0x04);
					  
				  }		  
				  break;
		      case RC_SW_UP:
			  {
					  control.CMControl.horizoltal        = 0;
					  control.CMControl.forwardOrbackward = 0; 
                      control.CMControl.left_right        = 0;	
				  
					  xEventGroupClearBits(BolletFreControlEnt,0x03);
                      xEventGroupClearBits(CmGimbalOutputEnt,0x04);
					  xEventGroupSetBits(CmGimbalOutputEnt,0x08); //��������ʱ������				  
			  }
				  break;
			  default :
				  break;
		  }
		  
		  if(control.GimbalControl.updown == 0)
		     limitFlag = 1;
		  
		  if(limitFlag == 1 )
			  {
				  if(control.GimbalControl.updown  > Pitch_LIMIT_ANGLE) //����P�����°ڵĽǶ�
					  control.GimbalControl.updown = (float)Pitch_LIMIT_ANGLE;
				  else if(control.GimbalControl.updown < - Pitch_LIMIT_ANGLE)
					  control.GimbalControl.updown = (float)(-Pitch_LIMIT_ANGLE);		  
			  }

		   
		  xQueueOverwrite(CMDataQueue,&control.CMControl);
		  xQueueOverwrite(GimbalDataQueue,&control.GimbalControl);	    	

		   if(364 == RemotData.rc.ch0 && 364 == RemotData.rc.ch3) //��8����Ħ����У׼�ź�
			   if(FrictionCalistop == 0)  xSemaphoreGive(FrictionCaliTriggerSem);	
			
			if(1684 == RemotData.rc.ch0 && 364 == RemotData.rc.ch3)//��8����Ħ����У׼�ź�
			   if(FrictionCalistop == 1) xSemaphoreGive(FrictionCaliSem);	
			
		   if(364 == RemotData.rc.ch0 && 1684 == RemotData.rc.ch2)  
			   if(ImuGimbalCalistop == 0)  xSemaphoreGive(ImuGimbalCaliTriggerSem);	
			
			if(1684 == RemotData.rc.ch0 && 364 == RemotData.rc.ch2) 
			   if(ImuGimbalCalistop == 1) xSemaphoreGive(ImuGimbalCaliSem);				
	}	
}


static  void Can1ReceiveTask(void *pvParameters)
{
	CanRxMsg rx_message; 
   for(;;)
	{
	   xQueueReceive(Can1ReceiveQueue,&rx_message,portMAX_DELAY);  
		switch(rx_message.StdId)
			{
				case 0x201:
					CM_Motor[0].getspeed = (rx_message.Data[2]<<8) | rx_message.Data[3];
					CM_Motor[0].onlinecnt = 0;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<4));
					break;
				case 0x202:
					CM_Motor[1].getspeed = (rx_message.Data[2]<<8) | rx_message.Data[3];
				    CM_Motor[1].onlinecnt = 0;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<5));
					break;	
				case 0x203:
					CM_Motor[2].getspeed = (rx_message.Data[2]<<8) | rx_message.Data[3];
				    CM_Motor[2].onlinecnt = 0;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<6));
					break;
				case 0x204:
					CM_Motor[3].getspeed = (rx_message.Data[2]<<8) | rx_message.Data[3];
				    CM_Motor[3].onlinecnt = 0;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<7));
					break;
				case 0x205:					
					YawMotor.position.encoder_p.currValue  =   (rx_message.Data[0]<<8)  | rx_message.Data[1];
				    YawMotor.onlinecnt    = 0;
					xEventGroupClearBits(OfflineDetectedEnt,(1<<2));
					break;
				case 0x206:
					PitchMotor.position.encoder_p.currValue = (rx_message.Data[0]<<8)  | rx_message.Data[1];
				    PitchMotor.onlinecnt  = 0 ;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<3));
					break;
				case 0x207:
					FireMotor.getspeed =  ((rx_message.Data[2]<<8)  | rx_message.Data[3]);
				    FireMotor.onlinecnt  = 0 ;
				    xEventGroupClearBits(OfflineDetectedEnt,(1<<8));					
				break;
//				case 0x501:
//					memcpy(&Imu_ext,rx_message.Data,5);
				default:
					
					break;
			}  
	}
}


static void Can2ReceiveTask(void *pvParameters)
{
	CanRxMsg rx_message;
   
   for(;;)
	{
		xQueueReceive(Can2ReceiveQueue,&rx_message,portMAX_DELAY); 
		
		switch(rx_message.StdId)
		{
//			case 0x501:
//				memcpy(&Imu_ext,rx_message.Data,5);
//			break;
			
			default :
				break;
		}
	  
	}
}

static void IMUReceiveTask(void *pvParameters)
{
   for(;;)
	{
		xSemaphoreTake(ImuDataSem,portMAX_DELAY);
	    GetPitchYawGxGyGz(&Mpu6500Data);	
	}
}


static void  ImuGimbalCaliTask(void *pvParameters)
{
	int32_t buff[5];
	int cnt  = 0;
	
	ImuGimbalCaliTriggerSem =  xSemaphoreCreateBinary();
	ImuGimbalCaliSem =  xSemaphoreCreateBinary();	
  for(;;)
  {
	  xSemaphoreTake(ImuGimbalCaliTriggerSem,portMAX_DELAY);
	  
	  xEventGroupClearBits(CmGimbalOutputEnt,0x03); //���õ��̺���̨������0
	  vTaskDelete(GimbalControlTask_Handler);
	  
	  ImuGimbalCalistop = 2;
	  FrictionCaliTriggerMusic();
	  do
	  {
		  MPU6500_InitGyro_offset(&Mpu6500Data);
		  GetPitchYawGxGyGz(&Mpu6500Data);
          cnt++;		  
	  }while(fabs(Mpu6500Data.Gyro_X) > 0.01 || fabs(Mpu6500Data.Gyro_Z) > 0.01 || cnt <10 );	
	  ImuCaliMusic();
	  ImuGimbalCalistop = 1;
     xSemaphoreTake(ImuGimbalCaliSem,portMAX_DELAY);
	 ImuGimbalCalistop = 2;	 
     /*���ĵĲ���������Ч*/
     YawMotor.position.encoderInitValue	     =   YawMotor.position.encoder_p.currValue;
	 PitchMotor.position.encoderInitValue	 =   PitchMotor.position.encoder_p.currValue;
	
	   memset(buff,0,sizeof(buff));
	  
	   buff[0]    = YawMotor.position.encoder_p.currValue;
	   buff[1]    = PitchMotor.position.encoder_p.currValue;
	   buff[2]    = Mpu6500Data.Gx_offset;
	   buff[3]    = Mpu6500Data.Gz_offset;
	   buff[4]    = Mpu6500Data.Gy_offset;
	  
  	 
	  
      /*���ĵĲ���������flash,�´ο�����Ч*/
      STMFLASH_Write(ADDR_FLASH_SECTOR_7,buff,sizeof(buff)/sizeof(buff[0])); 
	  FrictionCaliMusic(); 
      ImuGimbalCalistop = 0;	
	  
      xTaskCreate(GimbalControlTask,"gimbalControltask",512,NULL,4,&GimbalControlTask_Handler);  
  }	  
}


/*Ħ����У׼*
*У׼����Ϊ���Ȳ���Ħ���ֵ���ź��ߣ���Դ���Ȳ�Ҫ��
*Ȼ����ң����8�������������ߵ������� ����Ħ���ֵ����
*Դ�ߣ���������--�ε�--�ε���������������һ��ʱ��������
*ң����8����������������������������У׼
*/
static void FrictionCaliTask(void *pvParameters)
{
   
	FrictionCaliTriggerSem =  xSemaphoreCreateBinary();
	FrictionCaliSem =  xSemaphoreCreateBinary();
	for(;;)
	{
		xSemaphoreTake(FrictionCaliTriggerSem,portMAX_DELAY);
		
		vTaskDelete(FrictionControlTask_Handler);
		FrictionCalistop = 1;
		
		FrictionCaliTriggerMusic();
			
		FRICTION_MOTER_L  = 200;
		FRICTION_MOTER_R  = 200;	
		
		
		xSemaphoreTake(FrictionCaliSem,portMAX_DELAY);
		
		FrictionCalistop = 2;
		FrictionCaliMusic();
		
		 for(int i = 250;i>=100;i--)
		 {
			FRICTION_MOTER_L  =i;
			FRICTION_MOTER_R  =i;	
		 }	
		FrictionCalistop = 0;
        xTaskCreate(FrictionControlTask,"calitask",512,NULL,3,&FrictionControlTask_Handler);		 
	}
}

static void FrictionControlTask(void *pvParameters)
{

   EventBits_t ControlBits;
	
	TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(20);
	PreviousWakeTime = xTaskGetTickCount();
  for(;;)
	{
	   ControlBits = xEventGroupGetBits(BolletFreControlEnt);
			
	   switch(ControlBits)
	   {

           case 3: 
               SetFireSpeed(120); 
		       Set_Fire_Fre(135 * M2006_REDUCTIONRTION);
			   break;
           case 2:  		   
			   SetFireSpeed(120); 
		       Set_Fire_Fre(0);
		        break;
           case 1:  		   
			   SetFireSpeed(100); 
		       Set_Fire_Fre(135 * M2006_REDUCTIONRTION);
		        break;
		   case 0: 			
			   SetFireSpeed(100);   //Ħ���ֿ��� ������ֵ��Χ��100--200
		       Set_Fire_Fre(0);
               break;		   
		   default:
			   break;
	   }  
		vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);	   
	}  	
}
static void Set_Fire_Fre(int16_t fre)
{
	static int cnt = 0,cnt1 = 0;
	static char flag = 0;	
	int16_t fire_set_speed =  fre;

	if(fire_set_speed != 0)
	{
		if(	FireMotor.getspeed < 100)
		{
			 cnt++;
			 if(cnt > 40)
			 {
					flag = 1;
					cnt = 0;
			 }
		}			
	   if(flag == 1)
		 {
			 cnt1++;
			 fire_set_speed = -1500; //�����ת
			 if(cnt1 > 10)
			 {
				 flag = 0;
				 cnt1 = 0;
				 fire_set_speed = fre;
			 }
		 }	
	}
	FireMotor.setSpeed = fire_set_speed;	
}

int encoder_value_for_encoder(Motor_Posi_A_Speed_LoopData_t *input_data)
{
	input_data->position.encoder_p.previousValue = input_data->position.encoder_p.lastValue;
	input_data->position.encoder_p.lastValue     = input_data->position.encoder_p.currValue;
	
	if((input_data->position.encoder_p.lastValue - input_data->position.encoder_p.previousValue) < -ENCODER_MIDDLE)
		input_data->position.encoder_p.cnt++;
	if((input_data->position.encoder_p.lastValue - input_data->position.encoder_p.previousValue) >  ENCODER_MIDDLE)
		input_data->position.encoder_p.cnt--;
	
	return input_data->position.encoder_p.lastValue + input_data->position.encoder_p.cnt * 8191u;
}

static void ChassisControlTask(void *pvParameters)
{
	char L_Or_R = 0;
	EventBits_t ControlBits;
    TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(8);
	PreviousWakeTime = xTaskGetTickCount();
	
	struct CMControlValue controlvalue;
	
   for(;;)
	{
		xQueueReceive(CMDataQueue,&controlvalue,0);
	    ControlBits = xEventGroupGetBits(CmGimbalOutputEnt);
		if(ControlBits & 0x01 )
		{
			/*�ٽ紦����ֹ���̰ڶ�����180��*/
			if(YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.currValue < - ENCODER_MIDDLE)
				YawMotor.position.encoder_p.lastValue = YawMotor.position.encoder_p.currValue -  8192u;                                                                              
			else if(YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.currValue > ENCODER_MIDDLE)
				YawMotor.position.encoder_p.lastValue  = YawMotor.position.encoder_p.currValue + 8192u ;
			else
				YawMotor.position.encoder_p.lastValue = YawMotor.position.encoder_p.currValue;
			
			if(controlvalue.DnfanceAngle ==  0u) //��������
			{
			    controlvalue.left_right = ((YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.lastValue)*360u /8192u ) * CM_FOLLOW_SPEED;
				L_Or_R = 0;
			}	 
			else
			{
			   if(L_Or_R == 0) //������̬
			   {			
				   controlvalue.left_right = controlvalue.DnfanceAngle * controlvalue.DnfanceSpeed;
				   
			   	   if(((YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.lastValue) * 360.0f / 8192.0f) <=\
				      -controlvalue.DnfanceAngle)
				   {
				      L_Or_R = 1; 
				   }
			   }
			  else if(L_Or_R == 1)
			   {			
				   controlvalue.left_right = -  controlvalue.DnfanceAngle * controlvalue.DnfanceSpeed;
				   
			   	   if(((YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.lastValue) * 360.0f / 8192.0f) >=\
				      controlvalue.DnfanceAngle)
				   {
				      L_Or_R = 0; 
				   }
			   }			   
			}
			 CM_Motor[0].setSpeed =  controlvalue.forwardOrbackward + \
									 controlvalue.left_right	    + \
									 controlvalue.horizoltal;		
			 CM_Motor[1].setSpeed = -controlvalue.forwardOrbackward + \
									 controlvalue.left_right	    + \
									 controlvalue.horizoltal;	
			 CM_Motor[2].setSpeed = controlvalue.forwardOrbackward + \
									 controlvalue.left_right	   - \
									 controlvalue.horizoltal;			
			 CM_Motor[3].setSpeed = -controlvalue.forwardOrbackward + \
									 controlvalue.left_right	   - \
									controlvalue.horizoltal;		
		}
		else
		{
		   CM_Motor[0].setSpeed = 0;
		   CM_Motor[1].setSpeed = 0;
		   CM_Motor[2].setSpeed = 0;
		   CM_Motor[3].setSpeed = 0;
		}
	  vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);		
	}	
}
/*pitc������������˵��
*������ֵ��ʱ������
*��������Ϊ��ֵʱ�����˳ʱ��ת
*�����㣺�趨ֵ-����ֵ
*
*yaw������������˵��
*λ�û�KpΪ��ֵ
*�ٶȻ�KpΪ��ֵ
*IMU������ٶ�Ϊ�� 
*ң������Ϊ��
*IMU�Ƕ�����Ƕ�����
*
*pitcch�����������˵��
*λ�û�Kp -
*�ٶȻ�Kp +
*IMU���ϼ��ٶ�Ϊ�� 
*ң������Ϊ��ֵ

*����ͷ�����°�װ
*/
static void GimbalControlTask(void *pvParameters)
{
	struct GimbalControlValue controlvalue;
    char init_flag = 0;
	int16_t errY= 0,errP = 0;
	EventBits_t ControlBits,ControlBits_last = 0;
	
	int16_t last_y = 0,last_p = 0,diff_y = 0,diff_p = 0;
	
	TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(8);
	PreviousWakeTime = xTaskGetTickCount();
	
	xEventGroupSetBits(CmGimbalOutputEnt,0x02);	
	
    xEventGroupSetBits(CmGimbalOutputEnt,0x08);	
   for(;;)
	{	
	    #if(IMU_MODE == 1)
			YawMotor.speed.getspeed   =  -Mpu6500Data.Gyro_Z;
			PitchMotor.speed.getspeed =  Mpu6500Data.Gyro_X ;
		#else
			YawMotor.speed.getspeed   =  Imu_ext.Gyo_Z;
			PitchMotor.speed.getspeed =  Imu_ext.Gyo_X ;
		#endif
	
		xQueueReceive(GimbalDataQueue,&controlvalue,0);
		
	if(AutoData.YawAxiaAngle != 0)
	{
	    xEventGroupSetBits(CmGimbalOutputEnt,0x20); //��⵽Ŀ��
	}
	else
	{
	  xEventGroupClearBits(CmGimbalOutputEnt,0x20); //δ��⵽Ŀ��
	}
		
		if(init_flag == 0)
		{	
		    if(YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.currValue          < -ENCODER_MIDDLE)//�ٽ紦��
				 YawMotor.position.encoder_p.lastValue = YawMotor.position.encoder_p.currValue - 8192u;
			else if(YawMotor.position.encoderInitValue - YawMotor.position.encoder_p.currValue     >  ENCODER_MIDDLE)
				YawMotor.position.encoder_p.lastValue  = 8192u + YawMotor.position.encoder_p.currValue ;
			else
				YawMotor.position.encoder_p.lastValue = YawMotor.position.encoder_p.currValue;
			
		    if(PitchMotor.position.encoderInitValue - PitchMotor.position.encoder_p.currValue      < -ENCODER_MIDDLE)
				 PitchMotor.position.encoder_p.lastValue = PitchMotor.position.encoder_p.currValue - 8192u;
			else if(PitchMotor.position.encoderInitValue - PitchMotor.position.encoder_p.currValue >  ENCODER_MIDDLE)
				 PitchMotor.position.encoder_p.lastValue  = PitchMotor.position.encoder_p.currValue + 8192u ;			
			else
				PitchMotor.position.encoder_p.lastValue = PitchMotor.position.encoder_p.currValue;
			
			YawMotor.position.setPosition   =  YawMotor.position.encoderInitValue      * 360 /8192;
			YawMotor.position.getPosition   =  YawMotor.position.encoder_p.lastValue   * 360 /8192;
		    PitchMotor.position.setPosition =  PitchMotor.position.encoderInitValue    * 360 /8192;
			PitchMotor.position.getPosition =  PitchMotor.position.encoder_p.lastValue * 360 /8192;


			
			errY = YawMotor.position.setPosition    -  YawMotor.position.getPosition;
			errP = PitchMotor.position.setPosition  -  PitchMotor.position.getPosition;
			
			if(abs(errY) < 5 && abs(errP) < 5)
			{
			    init_flag = 1;
				Imu_ext.angle = 0;
			    Mpu6500Data.Angle_Yaw    = 0;
			    PitchMotor.position.encoder_p.cnt = 0;
				PitchMotor.position.encoder_p.lastValue = PitchMotor.position.encoder_p.currValue;
				PitchMotor.position.encoder_p.previousValue = 0;
                xEventGroupSetBits(CmGimbalOutputEnt,0x01);			
			}			
		}          
		else
		{
		        #if(IMU_MODE == 1)
					YawMotor.position.getPosition  = Mpu6500Data.Angle_Yaw;
				#else
					YawMotor.position.getPosition  = Imu_ext.angle;
				#endif
			
			PitchMotor.position.getPosition  = (encoder_value_for_encoder(&PitchMotor) * 360 /8192); 
			
			ControlBits = xEventGroupGetBits(CmGimbalOutputEnt); 			
			if((ControlBits & 0x10) == 0x10) //�����Զ�ģʽ
			{
				if((ControlBits_last & 0x10) == 0 ) //��¼����ǰ��λ��
				{
				    last_y = YawMotor.position.setPosition;
					last_p = PitchMotor.position.setPosition;
				}
				
			    if((ControlBits & 0x20) == 0x20) //����Ŀ��
				{
				   YawMotor.position.setPosition = 0 + 1.0;//ͨ����ֵ���趨Y��׼��λ��
				   YawMotor.position.getPosition = AutoData.YawAxiaAngle;		  
					
				   PitchMotor.position.setPosition = 0 + 1.0;//ͨ����ֵ���趨P��׼��λ��
				   PitchMotor.position.getPosition = -AutoData.PitchAxiaAngle;
				}
                else
				{
					if((ControlBits_last &0x20) == 0x20 ) 
					{
                        YawMotor.position.setPosition     =  YawMotor.position.getPosition;
                        PitchMotor.position.setPosition   =  PitchMotor.position.getPosition;						
					} 
				}					
			}
			else
			{
				if((ControlBits_last & 0x10) == 0x10 ) //�����л����ֶ�ģʽ��Ĳ�ֵ
				{
				     diff_y = YawMotor.position.getPosition   - last_y ;
					 diff_p = PitchMotor.position.getPosition - last_p ;
				}
				
				YawMotor.position.setPosition  = controlvalue.left_right + diff_y;
				
				PitchMotor.position.setPosition =  ((controlvalue.updown + diff_p +\
												     PitchMotor.position.encoderInitValue) * 360 /8192);						   
			}
			ControlBits_last = ControlBits;
		}			
		vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);
	}
}

int16_t  output[4];
int16_t  cmspeed[4];

static void PIDCalculateTask(void *pvParameters)
{	
	int16_t  yawPosiOutput= 0 ;
	int16_t  pitchPosiOutput = 0;
	EventBits_t ControlBits;
	int16_t maxout = 0;
	
	TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(4);
	PreviousWakeTime = xTaskGetTickCount();
	
    PID_Parms_Init();
	
	
	
  for(;;)
   {
	    ControlBits = xEventGroupGetBits(CmGimbalOutputEnt); 

        if((ControlBits & 0x8) == 0x8) //�����������
			{
			   maxout += 4;
				PitchMotor.position.pid.integral_limit = 100;
				PitchMotor.speed.pid.integral_limit    = 100;
				
				YawMotor.position.pid.integral_limit = 100;
				YawMotor.speed.pid.integral_limit    = 100;
				
			   YawMotor.speed.pid.max_out    = maxout;
			   PitchMotor.speed.pid.max_out  = maxout;
				
			   if(maxout >=4000) 
			   {
				   xEventGroupClearBits(CmGimbalOutputEnt,0x08);
				   YawMotor.speed.pid.max_out    = maxout;
			       PitchMotor.speed.pid.max_out  = maxout;
				   
				   PitchMotor.position.pid.integral_limit = 500;
				   PitchMotor.speed.pid.integral_limit    = 500;
				
				  YawMotor.position.pid.integral_limit = 500;
				  YawMotor.speed.pid.integral_limit    = 500;
				   
				  maxout =0;
			   }
			}	
	   
	    pitchPosiOutput =   (int16_t)pid_calc(&PitchMotor.position.pid,PitchMotor.position.setPosition,PitchMotor.position.getPosition);
	    PitchMotor.speed.setSpeed = pitchPosiOutput;
		//PitchMotor.speed.setSpeed  = 0; //�����ٶȻ�ʱʹ��
	    output[1]       =   (int16_t)pid_calc(&PitchMotor.speed.pid,PitchMotor.speed.setSpeed,PitchMotor.speed.getspeed);
	  
      
	   yawPosiOutput   =   (int16_t)pid_calc(&YawMotor.position.pid,YawMotor.position.setPosition,YawMotor.position.getPosition);	   
	  YawMotor.speed.setSpeed = yawPosiOutput;
	//  YawMotor.speed.setSpeed = 0;//�����ٶȻ�ʱʹ��
	   output[0]       =   (int16_t)pid_calc(&YawMotor.speed.pid,YawMotor.speed.setSpeed,YawMotor.speed.getspeed);
	   
	   output[2]       =   (int16_t)pid_calc(&FireMotor.pid,FireMotor.setSpeed,FireMotor.getspeed);
	   
	   for(char i = 0;i<4;i++)
	      cmspeed[i]   =   (int16_t)pid_calc(&CM_Motor[i].pid,CM_Motor[i].setSpeed,CM_Motor[i].getspeed);
       
	  if((ControlBits & 0x6) == 0x6)
	  {
		  CanSendMess(CAN1,SEND_ID205_207,output); 
		//  CanSendMess(CAN1,SEND_ID201_204,cmspeed); 	  
	  }
	  else	  
	  {
		  for(char i = 0;i<4;i++)
		  {
		    output[i]  = 0;
			cmspeed[i] = 0; 
		  }     
	      CanSendMess(CAN1,SEND_ID205_207,output); 
		  CanSendMess(CAN1,SEND_ID201_204,cmspeed); 
	  }   
	   CanSendMess(CAN2,SEND_ID201_204,cmspeed); 
     vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);
   }
}

static void OfflineDetectedTask(void *pvParameters)
{
	for(;;)
	{
	   Imu_ext.onlinecnt++;
	   RemotData.RC_onlineCnt++;
	   YawMotor.onlinecnt++;
	   PitchMotor.onlinecnt++;
	   FireMotor.onlinecnt++;
	   
	   for(char i = 0;i<4;i++)
		  CM_Motor[i].onlinecnt++;
   
	   if( Imu_ext.onlinecnt >=20)
	   {
		  xEventGroupSetBits(OfflineDetectedEnt,(1<<0));       
	   }
	   
	  if(RemotData.RC_onlineCnt > 20)
		{
		  xEventGroupSetBits(OfflineDetectedEnt,(1<<1)); 
		}
		
	  if(YawMotor.onlinecnt > 20)
	  {
		xEventGroupSetBits(OfflineDetectedEnt,(1<<2)); 
	  }
	  
	  if(PitchMotor.onlinecnt > 20)
	  {
		xEventGroupSetBits(OfflineDetectedEnt,(1<<3)); 
	  }
	  
	  if(FireMotor.onlinecnt > 20)
	  {
		xEventGroupSetBits(OfflineDetectedEnt,(1<<8)); 
	  }
	  
	   for(char i = 0;i<4;i++)
	     {
		    if(CM_Motor[i].onlinecnt > 20)
			{
			  xEventGroupSetBits(OfflineDetectedEnt,(1<<(4 +i)));  
			}
		 }  		 
	  vTaskDelay(10);
	}
}







