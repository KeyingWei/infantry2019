//
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                            O\ = /O
//                        ____/`---'\____
//                      .   ' \\| |// `.
//                       / \\||| : |||// \
//                     / _||||| -:- |||||- \
//                       | | \\\ - /// | |
//                     | \_| ''\---/'' | |
//                      \ .-\__ `-` ___/-. /
//                   ___`. .' /--.--\ `. . __
//                ."" '< `.___\_<|>_/___.' >'"".
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                 \ \ `-. \_ __\ /__ _/ .-` / /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//
//         .............................................
//                佛祖镇楼                 BUG辟易
//          ??:
//               					???????,???????;
//                 				???????,????????
//                 				???????,???????;
//                 				???????,????????
//                  			???????,???????;
//                  			???????,????????
//                  			???????,???????;
//                  			???????,????????
//				
//这里有佛祖，就问你怕不怕（滑稽）
/*----------------------------------------------------------------------------------------------
		移植的T型板的标准库代码，新更换的6623电机。云台参数需要重新整定，并且云台内外环方向需要重新整定
。就感觉6500貌似比6050稳定一点点，但是并不是太明显。运行蛮稳定的，自我感觉还行。
		比赛车太多了，全部宏定义出来比较好。
					--		新增基地
					--		英雄
					--		工程常
					--		新增摩擦轮闭环
					--		2310速度闭环（狗日的云台拨弹有点卡，导致2310速度闭环容易出问题）
					--		添加弹舱的舵机控制
					--		添加激光
					--		添加佛祖2333	
		--			╮(╯▽╰)╭			--
					
					2017.5.6
----------------------------------------------------------------------------------------------*/
/*-----------------------------------00000000000000000------------------------------------------

																			写注释 = 3 小时
																				
																				--╮(╯▽╰)╭--

----------------------------------------------------------------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__


/*------定义车辆工作或者读取云台机械角------*/
#define work   //work正常工作，read读取
//#define read 

/*------读取IPU漂移------*/
//#define IMU_Drift_Read

/*------定义车辆标识------*/
#define Car_Blue1


#ifdef  Car_Blue1


#define RM35_1_19
//#define RM35_1_27


#endif



#ifdef RM35_1_27
 
#define Forward_back_speed 11.6f
#define Left_right_spee 11.6f

#endif

#ifdef RM35_1_19
 
#define Forward_back_speed 9.6f
#define Left_right_speed 9.6f
#define	Rotate_speed 200
#endif


/*------RM6025 PID控制参数------*/
#define Pitch_Velocity_D  0 
#define Pitch_Position_I  0
#define Pitch_Position_D  0

//#define Yaw_Velocity_P  45//40
#define Yaw_Velocity_D  0 
//#define Yaw_Position_P  8.5f
#define Yaw_Position_I  0
#define Yaw_Position_D  0


/*------RM2310 PID控制参数------*/
#define ShootPosition_P 1
#define ShootPosition_I 0
#define ShootPosition_D 0
 
/*------底盘电机最大速度设置------*/
#define NORMAL_FORWARD_BACK_SPEED 			500
#define NORMAL_LEFT_RIGHT_SPEED   			700
#define HIGH_FORWARD_BACK_SPEED 			700
#define HIGH_LEFT_RIGHT_SPEED   			900  

/*------键盘鼠标速度设置------*/
//#define mouse_yaw_speed  1.5f
//#define mouse_pitch_speed - 0.084f

/*------P轴上下转动限位------*/


/*------拨弹电机速度------*/
//#define Shootspeed_input 2000

 
#include "sys.h"
#include "delay.h"


#include "can1.h"
#include "dbus.h"
#include "imu.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "math.h"
#include "usart.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "stm32f4xx_conf.h"




#include "pid.h"

#include "can2.h"
#include "pwm.h"
#include "timer.h"
#include "system.h"
#include "key.h"
#include "stmflash.h"



#include "math.h"
#define abs(x) ((x)>0? (x):(-(x)))
#define maxs(a,b) (a>b? a:b)

#endif

