#include "headfile.h"
/*************************************
������;remote_task
��ڲ�������
���ڲ�������
���ܣ�mydata.Cortol_ch0������ǰ�����ٶ�
      mydata.Cortol_ch1�� ���ƺ����ٶ�
			mydata.Cortol_ch2��������̨�����˶��ٶ�
			mydata.Cortol_ch3��������̨�����˶��ٶ�

*************************************/
void remote_task(void)
{
	mydata.Cortol_ch0 = (mydata.dbus.remote.ch0 - 1024) * 7;
	mydata.Cortol_ch1 = (mydata.dbus.remote.ch1 - 1024) * 7;
	mydata.Cortol_ch2 -= (mydata.dbus.remote.ch2 - 1024) * 0.0004f;
	mydata.Cortol_ch3 += (mydata.dbus.remote.ch3 - 1024) * 0.005f;
	
}


