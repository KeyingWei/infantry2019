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
//                ������¥                 BUG����
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
//�����з��棬�������²��£�������
/*----------------------------------------------------------------------------------------------
		��ֲ��T�Ͱ�ı�׼����룬�¸�����6623�������̨������Ҫ����������������̨���⻷������Ҫ��������
���͸о�6500ò�Ʊ�6050�ȶ�һ��㣬���ǲ�����̫���ԡ��������ȶ��ģ����Ҹо����С�
		������̫���ˣ�ȫ���궨������ȽϺá�
					--		��������
					--		Ӣ��
					--		���̳�
					--		����Ħ���ֱջ�
					--		2310�ٶȱջ������յ���̨�����е㿨������2310�ٶȱջ����׳����⣩
					--		��ӵ��յĶ������
					--		��Ӽ���
					--		��ӷ���2333	
		--			�r(�s���t)�q			--
					
					2017.5.6
----------------------------------------------------------------------------------------------*/
/*-----------------------------------00000000000000000------------------------------------------

																			дע�� = 3 Сʱ
																				
																				--�r(�s���t)�q--

----------------------------------------------------------------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__


/*------���峵���������߶�ȡ��̨��е��------*/
#define work   //work����������read��ȡ
//#define read 

/*------��ȡIPUƯ��------*/
//#define IMU_Drift_Read

/*------���峵����ʶ------*/
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


/*------RM6025 PID���Ʋ���------*/
#define Pitch_Velocity_D  0 
#define Pitch_Position_I  0
#define Pitch_Position_D  0

//#define Yaw_Velocity_P  45//40
#define Yaw_Velocity_D  0 
//#define Yaw_Position_P  8.5f
#define Yaw_Position_I  0
#define Yaw_Position_D  0


/*------RM2310 PID���Ʋ���------*/
#define ShootPosition_P 1
#define ShootPosition_I 0
#define ShootPosition_D 0
 
/*------���̵������ٶ�����------*/
#define NORMAL_FORWARD_BACK_SPEED 			500
#define NORMAL_LEFT_RIGHT_SPEED   			700
#define HIGH_FORWARD_BACK_SPEED 			700
#define HIGH_LEFT_RIGHT_SPEED   			900  

/*------��������ٶ�����------*/
//#define mouse_yaw_speed  1.5f
//#define mouse_pitch_speed - 0.084f

/*------P������ת����λ------*/


/*------��������ٶ�------*/
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

