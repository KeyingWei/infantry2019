#include "headfile.h"
#include "pid_modify.h"


u8 GimbalInit = 0;


/*************************************************************************
��������YawControlLoop_Init
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����Y���ٶȻ�����������
���ܣ��ϵ���̨Y����п���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
Y��������ṩλ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
float YawControlLoop_Init(GIMBALCONTORL *YawControl) //����
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;

	
	YawControl->Limit += 1;//�õ���ֵ��������ֹ����ʱ��������
	
	if(YawControl->Limit > 1500)		YawControl->Limit = 1500;
	
	 
	PositionOutPut = 	- (
													(
	                           YawControl->Encoder 
                           - YawControl->Encoder_Init
	                        )
                        	* 360 / 8192
	                    )  
	                         * YawControl->Position_kp;//Y��λ�û���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
	VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//���Ƶ������
	
				
	
	return yawCal;
}
static float yaw_ierror = 0;
/*************************************************************************
��������YawControlLoop_Work
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����Y���ٶȻ�����������
���ܣ���̨Y����������ʱ�Ŀ���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
������Z����ֵõ�λ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
float YawControlLoop_Work(GIMBALCONTORL *YawControl) //����
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	
	static const float Position_Ki = 0.001;
	
	static float perror = 0 ;
	
	
	 
	//yaw_ierror += perror;
	
	YawControl->Limit = 4500;

	//PositionOutPut = -(perror * YawControl->Position_kp + Position_Ki * yaw_ierror);//Y��λ�û�����
  if(Control_Mode == 0)  
	{
		perror = YawControl->ControlData + YawControl->Angle;
			PositionOutPut =  -(perror * YawControl->Position_kp );	
				
			VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ�����
				
			
	}
	else
	{
	  YawControl->ControlData = -YawControl->Angle ;
		
		if(AutoData.YawAxiaAngle != 0)
		PositionOutPut = (AutoData.YawAxiaAngle - 3.73f) * 8.2f;
		else
		AutoData.YawAxiaAngle = 0;
		
		
		VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * 55.0f;//Y���ٶȻ�����
			
	}

	  yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//���Ƶ���ֵ	
	return yawCal;
}
/*************************************************************************
��������YawControlCalc
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����
���ܣ���̨Y�����״̬ѡ��������Գ�ʼң��ֵ����
*************************************************************************/
void YawControlCalc(GIMBALCONTORL *YawControl)
{
	if(GetWorkState() != NORMAL_STATE )
	{
		mydata.Cortol_ch2 = mydata.imudata.Angle_Yaw = 0; //ң��ֵ����
		yaw_ierror = 0;
		
		YawControl->output = YawControlLoop_Init(YawControl);//�ϵ�õ�Y�����
	}
	else 
	{
	    YawControl->output = YawControlLoop_Work(YawControl);//��������ʱ��Y�����
	}
		
}

/*************************************************************************
��������PitchControlCalc
��ڲ�����
         GIMBALCONTORL *PitchControl������P��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�������
���ܣ��ϵ���̨P����п���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
P��������ṩλ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
void PitchControlCalc(GIMBALCONTORL *PitchControl) //����
{
		float PositionOutPut = 0.0f, VelocityOutPut = 0.0f;
		static float Encoder_init = 0, Encoder_E, Gimbal_MaxOutPut = 0;
		static u8 mode = 0;
		static int cout = 0;
	
	  static char angle2zero = 0;

		Gimbal_MaxOutPut += 1;
		if(Gimbal_MaxOutPut > 5000)		Gimbal_MaxOutPut = 5000;

	if(Control_Mode == 0)
	{
		 PositionOutPut = 	(
													(
															PitchControl->Encoder
															- PitchControl->Encoder_Init  	//��ʼ���Ļ�е�Ƕ�
															+ PitchControl->ControlData //������
													) 
													* 360 / 8192   //�Ƕ�ת��
											) 
											* PitchControl->Position_kp;
													
	
		VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * PitchControl->Velocity_kp;//P����ٶ�
	
	
	  if(VelocityOutPut > 5000)VelocityOutPut =5000;
		 if(VelocityOutPut < -5000)VelocityOutPut =-5000;
		 
		  
	}
	else
	{
	   PitchControl->ControlData =  Encoder_init- PitchControl->Encoder;
		
		if(AutoData.PitchAxiaAngle != 0)
		{
		  PositionOutPut = (AutoData.PitchAxiaAngle + 2.6) * 8.0f;
		}
		else
		{
		   PositionOutPut = 0;
		}
	   
	   VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * 35.0f;
	}
	
	 PitchControl->output =VelocityOutPut;	

	//	PitchControl->output = constrain(VelocityOutPut, -4500, 4500);
}


int16_t Get_angle(void)
{
	int16_t angle = 0;
   angle = (( PitchControl.Encoder	- PitchControl.Encoder_Init ) *360 /8192 ) ; 	//��ʼ���Ļ�е��
	
	 if(angle < -150)
		  angle += 360;
	 return angle;
}



int16_t FireControlLoop(int16_t SetSpeed)//����������� 
{
	   	CAN1_Motor_PID[6].set  =  SetSpeed;
	    pid_calc(&CAN1_Motor_PID[6]);	
	    CAN1_Motor_PID[6].out = constrain(CAN1_Motor_PID[6].out,-8000,8000);
	    return CAN1_Motor_PID[6].out;
	//return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.60f;
}


void Set_Fire_Fre(int16_t fre)
{
	static int cnt,   cnt1 = 0;
	static char flag = 0;
	
	int16_t fire_set_speed =  fre;

	if(fire_set_speed != 0)
	{
		if(	CAN1_Motor_PID[6].get < 100)
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
	
	FireControlLoop(fire_set_speed);
	
}



void Fire_speed_Control(int16_t speed)
{
    CAN2_Motor_PID[0].set =  -speed;
	  CAN2_Motor_PID[1].set =  speed;
	  pid_calc(&CAN2_Motor_PID[0] );
	  pid_calc(&CAN2_Motor_PID[1] );
	
	 CAN2_Motor_PID[0].out =  constrain(CAN2_Motor_PID[0].out,-7000,7000);
	 CAN2_Motor_PID[1].out =  constrain(CAN2_Motor_PID[1].out,-7000,7000);
 
}

void Set_fire_speed(int16_t speed) //����Ħ�����ٶ�
{
  Fire_speed_Control(speed);
}


void Set_Fire_Mode(Fire_Mode mode) 
{
	
	switch(mode)                                                       //fre: 88 :av:3�� speed:4500 av:13
		{                                                                 //  :2800 av:9,  speed:6800 av: 27
	   case H_SPEED_A_L_FRE:
			        Set_Fire_Fre(800); // ��Ƶ:2��/s
              Set_fire_speed(6500);  //���٣�
		          ShootMotor_Left = ShootMotor_Right = 150;
			 break;
	   case L_SPEED_A_H_FRE:
			        Set_Fire_Fre(2300); // ��Ƶ:8��/s
              Set_fire_speed(4500);;  //���٣�
		          ShootMotor_Left = ShootMotor_Right = 120;
			 break;
	   case M_SPEED_A_M_FRE:
			        Set_Fire_Fre(1500); // ��Ƶ:5��/s
              Set_fire_speed(5500); //���٣�
		           ShootMotor_Left = ShootMotor_Right = 130;
			 break;
	   case H_SPEED_A_H_FRE:
			        Set_Fire_Fre(2800); // ��Ƶ:8��/s
              Set_fire_speed(6500);  //���٣�
		          ShootMotor_Left = ShootMotor_Right = 150;
			 break;
		 case Fire_STOP:
			 			 	Set_Fire_Fre(0);
			 break;	 
	 }		 
}


void Heat2SetFire()  //���ݵȼ����������ò�����
{ 
  switch(Referee_date1.GameRobotState_t.robotLevel)
	{
		case LEVEL1:
			  if( Referee_date1.powerHeatData.shooterHeat0 > 90 - 25)
				{
				     Set_Fire_Mode(Fire_STOP); 
             					
				}
	//			else
	//			{
	//			   Set_Fire_Mode(H_SPEED_A_L_FRE);
	//			}
			break;
				
		case LEVEL2:
			  if( Referee_date1.powerHeatData.shooterHeat0 > ( 180 - 20))
				{
				     Set_Fire_Mode(Fire_STOP);   
			  }
//				else
//				{
	//			   Set_Fire_Mode(M_SPEED_A_M_FRE);
	//			}
			break;

	 case LEVEL3:
			  if( Referee_date1.powerHeatData.shooterHeat0 > ( 360 - 24))
				{
				     Set_Fire_Mode(Fire_STOP);   
				}
 	//			else
	//			{
	//			   Set_Fire_Mode(L_SPEED_A_H_FRE);
	//			}
			break;
				
	 default://Set_Fire_Mode(M_SPEED_A_M_FRE);
		 break;
	}		
}

