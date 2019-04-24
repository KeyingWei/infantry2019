#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "freertostask.h"   
#include "power_ctrl.h"
#include "BasicPeripherals.h"
#include "FrictionMoterPWM.h"
#include "RemotDbus.h"
#include "MainFocus_Usart.h"
#include "Referee.h"
#include "calibrate_task.h"
#include "start_task.h"


void  BSP_Init(void);
	
int main()
{

	//��ʼ���ײ�Ӳ��
	 BSP_Init();
	/*������ʼ���񲢿���������*/
     StartTask();
	/*Never arrive here*/
	while(1)
	{  
 	
   	}
}


//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void  BSP_Init()
{
	/*�������ȼ�����4����4λ��ռ���ȼ���0λ��Ӧ���ȼ�*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	
	//��ʼ���δ�ʱ��
    delay_init(180);		  
	//����24v�ɿص�Դ
	power_ctrl_configuration();	
	//��ʼ��led,beep,key,laser
	BasicPreiph_Init(); 
	//��ʼ��can1,can2
	MoterCanInit();
	//��ʼ��Ħ���ֿ���pwm
    FrictionMoterPWM_Init();
   //��ʼ��ң��DBUS	
	RemotDbus_Init();
	//��ʼ����ӡ����
	uart_init(115200);
   //��ʼ��������������ͨѶ	
	MainFocusConfig();
	//��ʼ���������ڲ���ϵͳͨѶ
	Referee_init();
	
	//��ʼ��MPU6500
    #if(IMU_MODE == 1)
	  IMU_Config();
     #endif	
	
	//24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
	//��flash�ж�ȡУ׼����
	Read_parma_from_flash();
}



