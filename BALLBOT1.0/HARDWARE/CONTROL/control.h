#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "math.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Encoder_Analysis(float Va,float Vb,float Vc);

int balance_Z(float Velocity,float Gyro);
int balance_X(float Angle,float Gyro);
int balance_Y(float Angle,float Gyro);
int velocity_X(int velocity);
int velocity_Y(int velocity);
		 				    
#endif
