#include "control.h"	


#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)    
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	      Target_A   = Vx + L_PARAMETER*Vz;
        Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	      Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
�������ܣ�С���˶� ���˶�ѧ���� ע��ʵ��ע�͵�����������˶�ѧ������ʵ��ʹ�÷Ŵ���3�����Լ�����ûӰ�죬��Ҫ�Ǳ�����ȥ���
��ڲ�����A B C����������ٶ�
����  ֵ����
**************************************************************************/
void Encoder_Analysis(float Va,float Vb,float Vc)
{
		compute_X=Va*2-Vb-Vc;
		compute_Y=(Vb-Vc)*sqrt(3);
		compute_Z=Va+Vb+Vc;
}


/**************************************************************************
�������ܣ�Z�����
��ڲ�����Z���ٶȡ�Z����ٶ�
����  ֵ��Z����ƿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance_Z(float Velocity,float Gyro)
{
   float Bias,Target;   //ƫ���Ŀ��ֵ
	 //static float Target_Velocity=600;//ң�ص��ٶ�
	 int balance,turn_kd;
/*
   if(Turn_Left==1)Target=Target_Velocity/sudu,turn_kd=0;   //������ת
	 else if(Turn_Right==1)Target=-Target_Velocity/sudu,turn_kd=0;
	 else
	*/
	 Target=0,turn_kd=Turn_Kd;
	
	 Bias=Velocity-Target;        //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Turn_Kp*Bias/10+Gyro*turn_kd/100;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ��  
	 return balance;
}
/**************************************************************************
�������ܣ�ֱ��PD����Y
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance_Y(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance_X(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 
	balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	// printf("Bias=%f",Bias);
	 //printf("Gyro=%f\r\n",Gyro);

	 return balance;
}
