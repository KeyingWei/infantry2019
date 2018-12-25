#include "headfile.h"

IncrementalPid ID201_PID = motor_id201;
IncrementalPid ID202_PID = motor_id201;
IncrementalPid ID203_PID = motor_id201;
IncrementalPid ID204_PID = motor_id201;
IncrementalPid ID207_PID = motor_id201;




/**************************************************************************************************
��������IncrementalPidCalc
��ڲ�����
				IncrementalPid * pid ������ʽPID�ṹ��ָ��
���ڲ�������
���ܣ�����һ���ṹ���ַ����������ֵ����������PID����
˵����
		����ʽPID���㹫ʽ��out = Kp *(e[n]-e[n-1])  +  Ki*e[n] +Kd*(e[n]  -  2*e[n-1]  +  e[n-2])
*******************************************************************************************************/
void IncrementalPidCalc(IncrementalPid * pid)
{
	  int16_t Error, pError, dError, incrementSpeed, output = 0;
		static int16_t Error_Last, Error_Next;
	
		Error = pid->SetSpeed - pid->NextSpeed;
		pError = Error - Error_Last;
		dError = Error - 2 * Error_Next + Error_Last;
	
		incrementSpeed = pid->Kp * pError 
												+ pid->Ki * Error 
														+ pid->Kd * dError;
		output += incrementSpeed;
		pid->output = constrain(output, -pid->Limit, pid->Limit);
		Error_Last = Error_Next;
		Error_Next = Error;
}

PositionPid ID207;
/**************************************************************************************************
��������PositionPidCalc
��ڲ�����
				PositionPid *pid ��λ��ʽPID�ṹ��ָ��
���ڲ�������
���ܣ�����һ���ṹ��ָ�룬��������ֵ����������PID����
˵����
		λ��ʽPID���㹫ʽ��out = Kp * (e[n]  +  Ki * sum��e[n]��  -  Kd*(e[n]  -  e[n-1]))
*******************************************************************************************************/
void PositionPidCalc(PositionPid *pid)
{
    static int16_t Error_Last;
    int16_t Error, iError, dError, output;
		
    Error = pid->SetPoint - pid->NextPoint;
		iError += Error;
		dError = Error - Error_Last;
		Error_Last = Error;
	
		output = Error * pid->Kp
								+ Error * pid->Ki 
										+ dError * pid->Kd;
	
    pid->output = constrain(output, -pid->Limit, pid->Limit);
		
}

/**************************************************************************************************
��������constrain
��ڲ�����
				1. int amt ��Ҫ�޷�����������
        2. int low ��Сֵ
        3. int high���ֵ 
���ڲ�����
				amt:�޷���Ĳ���
���ܣ� ����һ������,����������������ֵ��С����Сֵʱ�������������ֵ����Сֵ
*******************************************************************************************************/
int constrain(int amt, int low, int high)   //�޷�
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}


