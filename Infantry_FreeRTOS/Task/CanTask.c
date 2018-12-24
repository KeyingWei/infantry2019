#include "headfile.h"
#include "pid_modify.h"
static int16_t cout_p = 0, cout_y = 0;
static int16_t new_data_p = 4096, old_data_p;
static int16_t new_data_y = 4096, old_data_y;

static long int old_data_m,new_data_m = 4096,cout_m;

int16_t watch =0,watch1=0;;

/* Get the value of the continuous encoder of the pitch axis */
/*************************************************************************
��������encoder_value_for_pitch
��ڲ�����
         input_data��P����յ��Ļ�е��
���ڲ�������
���ܣ���P��Ļ�е�ǽ�������������
˵����������ֵ���ٽ�ʱ�������ϸ�������10,�¸�������-8000��˵���Ѿ������ٽ�ǣ�
�������������������õ�������ֵ�Ǵ�ģ�����������ķ����ǵ������ٽ��ʱ��˳
ʱ�뾭���ٽ��ʱ��Ȧ����1����ʱ��ת��ʱ��1
*************************************************************************/
int encoder_value_for_pitch(int16_t input_data)  
{
	old_data_p = new_data_p;
	new_data_p = input_data;
	
	if((new_data_p - old_data_p) < -5200)
		cout_p++;
	else if((new_data_p - old_data_p) > 5200)
		cout_p--;
	watch1= cout_p;
	return new_data_p + cout_p * 8191;
}

/* Get the value of the continuous encoder of the yaw axis */
/*************************************************************************
��������encoder_value_for_yaw
��ڲ�����
         input_data��Y����յ��Ļ�е��
���ڲ�������
���ܣ���Y��Ļ�е�ǽ�������������
*************************************************************************/
int encoder_value_for_yaw(int16_t input_data)
{
	old_data_y = new_data_y;
	new_data_y = input_data;
	
	if((new_data_y - old_data_y) < -5200)
		cout_y++;
	else if((new_data_y - old_data_y) > 5200)
		cout_y--;
	watch= cout_y;
	return new_data_y + cout_y * 8191;
}

long int encoder_value_for_2006(int16_t input_data)
{
	old_data_m = new_data_m;
	new_data_m = input_data;
	
	if((new_data_m - old_data_m) < -7000)
		cout_m++;
	else if((new_data_m - old_data_m) > 7000)
		cout_m--;
	return (new_data_m + cout_m * 8191);
}

/* Get the motor all the motor parameters */
/*************************************************************************
��������ReceiveCAN1Data
��ڲ�������
���ڲ�������
���ܣ����ݵ��ID���յ����ת�ټ���е������
ע�⣺
�������Ƶ��1kHz��
��е�Ƕ�ֵ�ķ�Χ��0~8191(0x1FFF)
ʵ�ʵ�������ֵ��Χ��-13000 ~ 13000
*************************************************************************/
int origion_yaw_encoder_val  =0;
int origion_pith_encoder_val =0;
int init_flag = 0;

void ReceiveCAN1Data(MyData *mydata)
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
	
	switch(rx_message.StdId)
	{	
		case CAN_BUS1_MOTOR1:
		{             
			global_cout[LOST_COUNTER_INDEX_MOTOR1] = Online;
			CAN1_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
		}
		break;
		
		case CAN_BUS1_MOTOR2:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR2] = Online;
			CAN1_Motor_PID[1].get= (rx_message.Data[2]<<8) + rx_message.Data[3];
		}	
		break;
			
		case CAN_BUS1_MOTOR3:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR3] = Online;
			CAN1_Motor_PID[2].get = (rx_message.Data[2]<<8) + rx_message.Data[3];			
		}
		break;
			
		case CAN_BUS1_MOTOR4:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR4] = Online;
			CAN1_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
		}
		break;			
		
		case CAN_BUS1_MOTOR5:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR5] = Online;
			origion_yaw_encoder_val = (rx_message.Data[0] << 8) | rx_message.Data[1];
			
			if(workState == PREPARE_STATE)
			{
			   if(origion_yaw_encoder_val -  YawControl.Encoder_Init > 5000)
				 {
				    mydata->usemotor.rm6623_y.encoder = origion_yaw_encoder_val - 8192;  
				 }
				 else if(origion_yaw_encoder_val -  YawControl.Encoder_Init < -5000)
				 {
				    mydata->usemotor.rm6623_y.encoder = origion_yaw_encoder_val + 8192; 
				 }
				 else
				 {
				      mydata->usemotor.rm6623_y.encoder  = origion_yaw_encoder_val ; 
				 }
			}
			else
			{
			   mydata->usemotor.rm6623_y.encoder = encoder_value_for_yaw(origion_yaw_encoder_val);//����ֵ���������� 
			}

		}
		break;
		
		case CAN_BUS1_MOTOR6:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR6] = Online;
			origion_pith_encoder_val  = (rx_message.Data[0] << 8) | rx_message.Data[1];
			
			
		if(workState == PREPARE_STATE)
			{
			   if(origion_pith_encoder_val -  PitchControl.Encoder_Init > 5000)
				 {
				    mydata->usemotor.rm6623_p.encoder = origion_pith_encoder_val - 8192;  
				 }
				 else if(origion_pith_encoder_val -   PitchControl.Encoder_Init  < -5000)
				 {
				    mydata->usemotor.rm6623_p.encoder = origion_pith_encoder_val + 8192; 
				 }
				 else
				 {
				      mydata->usemotor.rm6623_p.encoder  = origion_pith_encoder_val  ; 
				 }
			}
			else
			{
			  mydata->usemotor.rm6623_p.encoder = encoder_value_for_pitch(origion_pith_encoder_val);//����ֵ����������
			}

		}
		break;
		
		case CAN_BUS1_MOTOR7:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR7] = Online; 
			mydata->usemotor.rm2310_fire.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
			CAN1_Motor_PID[6].get =  (rx_message.Data[2] << 8) | rx_message.Data[3];
			CAN1_Motor_Position_PID[0].position_get = ((encoder_value_for_2006((rx_message.Data[0] << 8) | rx_message.Data[1]))/ 36);	
			if(init_flag == 0)
			{
			   CAN1_Motor_Position_PID[0].init_position  = CAN1_Motor_Position_PID[0].position_get; 
				 init_flag = 1;
			}
			
			
		}
		break;
		
		default:break;
	}
}

/*       CAN1_RX0_IRQHandler           */
/*************************************************************************
��������CAN1_RX0_IRQHandler
��ڲ�������
���ڲ�������
���ܣ�CAN�����жϽ��յ�����͵Ļ�е�Ǽ�ת������
ע�⣺
�������Ƶ��1kHz��
��е�Ƕ�ֵ�ķ�Χ��0~8191(0x1FFF)
ʵ�ʵ�������ֵ��Χ��-13000 ~ 13000
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		ReceiveCAN1Data(&mydata);
	}
}

void CAN1_TX_IRQHandler()
{
  if(CAN_GetITStatus(CAN1,CAN_IT_TME) !=RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	 }
}
/****************************************************************************************************************************
����ԭ�ͣ�void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
��ڲ�����
1.CAN_TypeDef *CANx ָ������CAN1���Ǵ�CAN2
2. int16_t gimbal_yaw_iq  ��̨IDΪ205��Y�������͵�ת�ص�������
3. int16_t gimbal_pitch_iq��̨IDΪ206��Y�������͵�ת�ص�������
4. int16_t gimbal_fire_iq ��̨IDΪ207�Ĳ���������͵�ת�ص�������
5.int16_t cm4_iq ����IDΪ204�ĵ�����͵�ת�ص�������
���ڲ�������
���ܣ�������̨�����ת�ص�������
*****************************************************************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
{
    CanTxMsg tx_message;  

      
    tx_message.StdId = 0x1FF;//���ͱ�׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//����֡����8���ֽڵ�����
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(gimbal_fire_iq >> 8);
    tx_message.Data[5] = (unsigned char)gimbal_fire_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************************************************
����ԭ�ͣ�void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
��ڲ�����
1.CAN_TypeDef *CANx ָ������CAN1���Ǵ�CAN2
2.int16_t cm1_iq ����IDΪ201�ĵ�����͵�ת�ص�������
3.int16_t cm2_iq ����IDΪ202�ĵ�����͵�ת�ص�������
4.int16_t cm3_iq ����IDΪ203�ĵ�����͵�ת�ص�������
5.int16_t cm4_iq ����IDΪ204�ĵ�����͵�ת�ص�������
���ڲ�������
���ܣ����͵��̵����ת�ص�������
**************************************************************************************************************/

void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//��׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//���Ͱ˸��ֽڵ�����
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//���ݸ߰�λ
    tx_message.Data[1] = (uint8_t)cm1_iq;         //���ݵͰ�λ
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
	
	
    CAN_Transmit(CANx,&tx_message);
}


void ReceiveCAN2Data()
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN2, CAN_FIFO0, &rx_message);     
	switch(rx_message.StdId)
	{	
		case CAN_BUS2_MOTOR1:
		{             
			global_cout[LOST_COUNTER_INDEX_MOTOR8] = Online;
      CAN2_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
		}
		break;
		
		case CAN_BUS2_MOTOR2:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR9] = Online;
			CAN2_Motor_PID[1].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
				case CAN_BUS2_MOTOR3:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR10] = Online;
			CAN2_Motor_PID[2].get= (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
				case CAN_BUS2_MOTOR4: 
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR11] = Online;
			CAN2_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
		
		default:break;
	}
}


void CAN2_RX0_IRQHandler(void)
{   
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
    ReceiveCAN2Data();
	}
	
}


void CAN2_TX_IRQHandler()
{
		if(CAN_GetITStatus(CAN2,CAN_IT_TME) !=RESET)
		{
			 CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		 }  
}


void CAN2_Set_FireSpeed( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	  int i=0;
	  u8 mbox;
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//��׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//���Ͱ˸��ֽڵ�����
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//���ݸ߰�λ
    tx_message.Data[1] = (uint8_t)cm1_iq;         //���ݵͰ�λ
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    mbox = CAN_Transmit(CAN2,&tx_message);
	
	// while(CAN_TransmitStatus(CAN2,mbox) ==CAN_TxStatus_Failed && i++<0xffff);

}
