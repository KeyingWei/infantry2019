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

	//初始化底层硬件
	 BSP_Init();
	/*创建开始任务并开启调度器*/
     StartTask();
	/*Never arrive here*/
	while(1)
	{  
 	
   	}
}


//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void  BSP_Init()
{
	/*设置优先级分组4，即4位抢占优先级，0位响应优先级*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	
	//初始化滴答定时器
    delay_init(180);		  
	//配置24v可控电源
	power_ctrl_configuration();	
	//初始化led,beep,key,laser
	BasicPreiph_Init(); 
	//初始化can1,can2
	MoterCanInit();
	//初始化摩擦轮控制pwm
    FrictionMoterPWM_Init();
   //初始化遥控DBUS	
	RemotDbus_Init();
	//初始化打印串口
	uart_init(115200);
   //初始化串口用于妙算通讯	
	MainFocusConfig();
	//初始化串口用于裁判系统通讯
	Referee_init();
	
	//初始化MPU6500
    #if(IMU_MODE == 1)
	  IMU_Config();
     #endif	
	
	//24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
	//从flash中读取校准参数
	Read_parma_from_flash();
}



