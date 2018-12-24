#ifndef __MOTORCAN_H_
#define __MOTORCAN_H_
#include "stm32f4xx.h"
#include "pid_modify.h"

#define SEND_ID201_204 0x200
#define SEND_ID205_207 0X1FF
#define M2006_REDUCTIONRTION 36


typedef struct Motor_SpeedLoopData
{
	int16_t getspeed;  			 
	int16_t setSpeed;	
	pid_t pid;
	int16_t onlinecnt;
}Motor_SpeedLoopData_t;

struct Encoder
{
  int16_t cnt;
  int16_t lastValue;
  int16_t previousValue;
  int16_t currValue;
};

typedef struct  Motor_Posi_LoopData
{ 
    struct Encoder	encoder_p;
	int16_t encoderInitValue;
	int16_t setPosition;
	int16_t getPosition;
	pid_t pid;
}Motor_Posi_LoopData_t ;

typedef struct  Motor_Posi_A_Speed_LoopData
{
	struct  Motor_Posi_LoopData position;
	struct Motor_SpeedLoopData speed;
	int16_t onlinecnt;
}Motor_Posi_A_Speed_LoopData_t;

void MoterCanInit(void);
void CanSendMess(CAN_TypeDef* CANx,uint32_t SendID,int16_t *message);

#endif

