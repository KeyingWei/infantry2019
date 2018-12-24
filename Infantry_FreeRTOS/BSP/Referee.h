#ifndef __REFEREE_H
#define __REFEREE_H

#include "stdio.h"
#include "sys.h"
#include "usart.h"
#define AHRS_RC_LEN  120 	//定义接收字节数 68

#define ROBOT_STATE 1
#define HURTDATA    2
#define SHOOTDATA   3
#define POWER_HEAT  4
#define RFIDDETECT  5
#define GAMERUSULT  6
#define GETBUFF     7
#define POSITION    8

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;


typedef __packed struct 
{
	 uint16_t stageRemianTime;
	 uint8_t gameProgress;
	 uint8_t  robotLevel;
	 uint16_t remainHP;
	 uint16_t maxHP;
}extGameRobotState_t;

typedef __packed struct 
{
  uint8_t armorType :4;
	uint8_t hurtType  :4;
}extRoboHurt_t;

typedef __packed struct 
{
  uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
}extShootData_t;

typedef __packed struct
{
  float chassisVolt;
	float chassisCurrent;
	float chassisPower;
	float chassisPowerBuffer;
	
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
}extPowerHeatData_t;

typedef __packed struct
{
	uint8_t card_Type;
	uint8_t cardidx;
}extRfidDetect_t;

typedef __packed struct
{
   uint8_t winner;
}extGameResult_t;

typedef __packed struct 
{
	uint8_t buffType;
	uint8_t buffAddition;
}extGetBuff_t;

typedef __packed struct
{
  float x;
	float y;
	float z;
	float yaw;
}extGameRobotPos_t;

typedef struct __Referee_Date__
{		
		frame_header_t    frame_header;
		int16_t CmdID;	  
	  extGameRobotState_t GameRobotState_t;
	  extRoboHurt_t       RoboHurt_t;
	  extShootData_t      ShootData;
	  extPowerHeatData_t  powerHeatData;
	  extRfidDetect_t     RfidDetect;
	  extGameResult_t     GameResult;
		extGetBuff_t        GetBuff;
		extGameRobotPos_t   GameRobotPos;
	
}Referee_Date;


typedef struct _power
{
	volatile  int16_t SetPower;
	float curr_power;
	int16_t Limit;
	float Kp;	
	float Ki;
	float Kd;
  volatile  float average ;	
	volatile  int16_t output;
	float pout;
	float iout;
	float dout;
	float error[3];
}Power;


extern  Referee_Date Referee_date1;
extern  u16 Usart1_RX_Cou  ;
extern u8 Usart1_RX_Flag  ;
extern char  g_Referee_flag ;
extern Power power ; 

void Referee_init(void);

#endif
