#include "control.h"	
#include "filter.h"
#include "motor.h"

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
	 Target=0,
turn_kd=Turn_Kd;
	
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
   float Bias,kd_out;
	 int balance;
	 float pid_kp,pid_kd;
	 Bias=Angle;        //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 kd_out=Angle_Balance_Y_last-Bias;
	 pid_kp=Balance_Kp*Bias;
	 pid_kd=Gyro*Balance_Kd/100;
	 //pid_kd=Balance_Kd*kd_out;
	 balance=pid_kp+pid_kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	
//	 printf("pid_kp_y=%.1f",pid_kp);
//	 printf("pid_kd_y=%.1f\r\n",pid_kd);
	
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
	 float pid_kp,pid_kd;
	 float kd_out;
	 Bias=Angle;        //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 kd_out=Angle_Balance_Y_last-Bias;
	 pid_kp=Balance_Kp*Bias;
	 pid_kd=Gyro*Balance_Kd/100;
	 //pid_kd=Balance_Kd*kd_out;
	 balance=pid_kp+pid_kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
//	 printf("pid_kp_x=%.1f",pid_kp);
//	 printf("pid_kd_x=%.1f\r\n",pid_kd);
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity
��ڲ����������ٶȡ������ٶ�
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity_X(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	/*
		if(1==Flag_Left)    	Movement=-Target_Velocity;	           //===ǰ����־λ��1 
		else if(1==Flag_Right)	Movement=Target_Velocity;           //===���˱�־λ��1
  	else*/  
	Movement=0;
    //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter_X(velocity);        //�ٶ��˲�  
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
 		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                      //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;               //===�����޷�
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===�����޷�	
	  //if(Flag_Stop)   Encoder_Integral=0; //===����رպ��������
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;        //===�ٶȿ���	
	  //if(Flag_Stop)   Velocity=0;      //===����رպ��������
		if(Velocity>1000)  	Velocity=1000;               //===�ٶȻ��޷�
		if(Velocity<-1000)	  Velocity=-1000;              //===�ٶȻ��޷�
	  return Velocity;
}
/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity
��ڲ����������ٶȡ������ٶ�
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity_Y(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	/*
	  if(1==Flag_Qian)    	  Movement=Target_Velocity;	           //===ǰ����־λ��1 
		else if(1==Flag_Hou)	  Movement=-Target_Velocity;           //===���˱�־λ��1
  	else
  */  
	Movement=0;
//   //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter_Y(velocity);          //�ٶ��˲�      
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                  //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;            //===�����޷�
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===�����޷�	
    //if(Flag_Stop)   Encoder_Integral=0;      //===����رպ��������
  	Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;      //===�ٶȿ���	
	  //if(Flag_Stop)   Velocity=0;      //===����رպ��������
	  if(Velocity>1000)  	Velocity=1000;               //===�ٶȻ��޷�
		if(Velocity<-1000)	  Velocity=-1000;              //===�ٶȻ��޷�
	  return Velocity;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C;
			Final_Motor_A=Linear_Conversion(motor_a);  //���Ի�
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
	    if(motor_a<0) Final_Motor_A=-Final_Motor_A;
		  if(motor_b<0) Final_Motor_B=-Final_Motor_B;
	    if(motor_c<0) Final_Motor_C=-Final_Motor_C;

			set_motor(Final_Motor_A,Final_Motor_B,Final_Motor_C);  
}
/**************************************************************************
�������ܣ��Կ��������PWM���Ի�,���ڸ�ϵͳ�Ĵ�����ֵ
��ڲ�����PWM
����  ֵ�����Ի����PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/my_abs(motor);   //1000000�Ǿ���ֵ
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;			
}
u32 my_abs( int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
