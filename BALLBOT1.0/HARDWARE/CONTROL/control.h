#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "math.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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
