#include "headfile.h"
#include <math.h>
#include "pid_modify.h"


CMCONTORL     CmCotrol     = CMcontrolconfig;//���̿��Ʋ�����ֵ 
GIMBALCONTORL YawControl   = yawcontrolconfig; //��̨Y����Ʋ�����ֵ
GIMBALCONTORL PitchControl = pitchcontrolconfig;//��̨P����Ʋ�����ֵ

 uint32_t  time_1ms = 0;  
 

volatile int16_t g_fire_speed = 0; 

WorkState_e workState     = PREPARE_STATE;//�ϵ��ʼ��״̬��4-5s����
WorkState_e lastWorkState = PREPARE_STATE;

void Gimbal_Params_Init()
{
	YawControl.Encoder_Init = 7909  ;//��ȡ��ʼ����ֵ R2:7787  R3:6313
	  YawControl.Position_kp = 8.5f;//�趨λ�û����Ʋ���
	  YawControl.Velocity_kp = 42.0f;//�趨�ٶȻ����Ʋ���
	
	PitchControl.Encoder_Init = 7205     ;  // R2:6955   R3:409          ;	
	  PitchControl.Position_kp = 9.5f;
	  PitchControl.Velocity_kp = 48.0f;	
}


void WorkStateFSM(void);

WorkState_e GetWorkState(void)//��ȡ����״̬
{
		return workState;
}

void Set_WorkState(WorkState_e state)
{
   workState  = state ;
}


static void WorkStateSwitchProcess(void)
{
    if(lastWorkState != workState  && workState == PREPARE_STATE)
		{
		     time_1ms = 0;
		}
}
/***************************************************
������:CMControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡң�ز���ֵ����̨Y������ֵ�����µ��̿��Ʋ���
**************************************************/
void CMControlLoop(void)
{
	static char dance_flag = 0; 
	
	CM_Motor_Pidparams_init();
	CmCotrol.Control_CH0 = mydata.Cortol_ch0;//���Ƶ���ǰ���ƶ�
	CmCotrol.Control_CH1 = mydata.Cortol_ch1;//���Ƶ������Һ���
	CmCotrol.Encoder = mydata.usemotor.rm6623_y.encoder;//��ȡY������ֵ���������������������ڼ���ǶȲ�ֵ��ʵ�ֵ��̸���
	CmCotrol.Encoder_Init = YawControl.Encoder_Init;//��ȡ��̨��ʼ��λ��
 
	if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_R)
	{
	   dance_flag  = 1;
	}
	else if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_F)
	{
	   dance_flag  = 0;
	}
	
	if(dance_flag == 1)
	{
	   CmCotrol.Calc(1);
	}
	else
	{
	  CmCotrol.Calc(0);
	}
		
	
}
/***************************************************
������:YawControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡY��������ֵ��������Z����ٶȣ�Z����������
��ֵ����д�������
**************************************************/
void YawControlLoop(void)
{
	YawControl.Angle = mydata.imudata.Angle_Yaw; //��ȡ����Ƕ�
	YawControl.Gyro = mydata.imudata.Gyro_Z;//��ȡ������ٶ�
	
	YawControl.ControlData = mydata.Cortol_ch2;//��ȡ�������ֵ
	
	YawControl.Encoder = mydata.usemotor.rm6623_y.encoder;//��ȡY������ֵ

	YawControl.Calc(&YawControl);//��̨Y�����
}
/***************************************************
������:PitchControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡP��������ֵ��������X����ٶ�
��ֵ����д�������
**************************************************/
void PitchControlLoop(void)
{
	PitchControl.Gyro = mydata.imudata.Gyro_X;
	
	PitchControl.ControlData  = mydata.Cortol_ch3;
	PitchControl.Encoder      = mydata.usemotor.rm6623_p.encoder;
	
	PitchControl.Calc(&PitchControl);
}

u8 start =	1;
 
volatile long int fire_fre = 0;
 /***************************************************
������:GimbalMotorOutput
��ڲ�������
���ڲ�������
���ܣ� ����״̬������̨�������ң��ֵ�޷����ջ�ģ������ʱ����
̨�����������
**************************************************/
void GimbalMotorOutput(void)
{
	if(start == 1)
	{
		
			
	  WorkStateFSM();
		WorkStateSwitchProcess();
		
		YawControlLoop();
		PitchControlLoop();
				
		CMControlLoop();
		
		
		if( (Dog.rc == Lose) ||(GetWorkState() == STOP_STATE))
		{
			  Set_Gimbal_Current(CAN1, 0, 0, 0);
        Set_CM_Speed(CAN1,0,0,0,0);		
        CAN2_Set_FireSpeed(0,0,0,0);	
			ShootMotor_Left = ShootMotor_Right = 100;
		}
		else 
		{		
			Set_Gimbal_Current(CAN1, YawControl.output , PitchControl.output,CAN1_Motor_PID[6].out); //��̨��� R2:+ + +;R3 - + +
			Set_CM_Speed(CAN1,CAN1_Motor_PID[0].out, CAN1_Motor_PID[1].out, CAN1_Motor_PID[2].out, CAN1_Motor_PID[3].out); //�������
		//	CAN2_Set_FireSpeed(-CAN2_Motor_PID[0].out,CAN2_Motor_PID[1].out,0,0);//����Ħ���ַ���
		}
	}		
}



void WorkStateFSM(void)
{
    lastWorkState =   workState;
	  static char cali_success_flag = 0;
    static char cnt = 0,cnt1 = 0;
	 switch(workState) 
	 {
		 case PREPARE_STATE:
			   time_1ms ++;
			   if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 
				 }
				 else if(time_1ms > PREPARE_TIME_TICK_MS)
				 {
					  workState =  NORMAL_STATE;
            time_1ms = 0;				 
				 }
			 break;
		 case STANDBY_STATE:
			  if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;			 
				 }
			 
		 break;
		 case  NORMAL_STATE:
			 if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 }
			 break;
		 
		 case STOP_STATE:
		     //  time_1ms = 0 ;
			     if(GetInputMode() != STOP)
					 {
					     workState =  PREPARE_STATE;  
						  //   workState = CALI_STATE;						   
					 }
			 break;
		 case CALI_STATE:
  /*          do{
						  	MPU6500_InitGyro_offset();
							  delay_ms(10);
							  cnt1++;
                if(gy_data > -3 && gy_data  < 3)
								{
								   cnt++;
								   cali_success_flag = 0;
									 if(cnt >=6)
									 {
									    cnt = 0;
										  cali_success_flag = 1;
									 }										 
								}						     							  
						  }			 
          while(cali_success_flag == 0 && cnt1 < 100);
					workState =  PREPARE_STATE; 
          time_1ms = 0;								
					cnt1 = 0; */
			 break;
		 default:break;
	 }
	
}

