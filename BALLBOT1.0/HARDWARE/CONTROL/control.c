#include "control.h"	


#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)    
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	      Target_A   = Vx + L_PARAMETER*Vz;
        Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	      Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
函数功能：小车运动 正运动学分析 注：实际注释掉的是理想的运动学分析，实际使用放大了3倍，对计算结果没影响，主要是避免舍去误差
入口参数：A B C三个电机的速度
返回  值：无
**************************************************************************/
void Encoder_Analysis(float Va,float Vb,float Vc)
{
		compute_X=Va*2-Vb-Vc;
		compute_Y=(Vb-Vc)*sqrt(3);
		compute_Z=Va+Vb+Vc;
}


/**************************************************************************
函数功能：Z轴控制
入口参数：Z轴速度、Z轴角速度
返回  值：Z轴控制控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance_Z(float Velocity,float Gyro)
{
   float Bias,Target;   //偏差和目标值
	 //static float Target_Velocity=600;//遥控的速度
	 int balance,turn_kd;
/*
   if(Turn_Left==1)Target=Target_Velocity/sudu,turn_kd=0;   //控制自转
	 else if(Turn_Right==1)Target=-Target_Velocity/sudu,turn_kd=0;
	 else
	*/
	 Target=0,turn_kd=Turn_Kd;
	
	 Bias=Velocity-Target;        //===求出平衡的角度中值 和机械相关
	 balance=Turn_Kp*Bias/10+Gyro*turn_kd/100;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数  
	 return balance;
}
/**************************************************************************
函数功能：直立PD控制Y
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance_Y(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance_X(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===求出平衡的角度中值 和机械相关
	 
	balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	// printf("Bias=%f",Bias);
	 //printf("Gyro=%f\r\n",Gyro);

	 return balance;
}
