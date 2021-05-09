#include "control.h"	
#include "filter.h"
#include "motor.h"

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
	 Target=0,
turn_kd=Turn_Kd;
	
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
   float Bias,kd_out;
	 int balance;
	 float pid_kp,pid_kd;
	 Bias=Angle;        //===求出平衡的角度中值 和机械相关
	 kd_out=Angle_Balance_Y_last-Bias;
	 pid_kp=Balance_Kp*Bias;
	 pid_kd=Gyro*Balance_Kd/100;
	 //pid_kd=Balance_Kd*kd_out;
	 balance=pid_kp+pid_kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	
//	 printf("pid_kp_y=%.1f",pid_kp);
//	 printf("pid_kd_y=%.1f\r\n",pid_kd);
	
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
	 float pid_kp,pid_kd;
	 float kd_out;
	 Bias=Angle;        //===求出平衡的角度中值 和机械相关
	 kd_out=Angle_Balance_Y_last-Bias;
	 pid_kp=Balance_Kp*Bias;
	 pid_kd=Gyro*Balance_Kd/100;
	 //pid_kd=Balance_Kd*kd_out;
	 balance=pid_kp+pid_kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
//	 printf("pid_kp_x=%.1f",pid_kp);
//	 printf("pid_kd_x=%.1f\r\n",pid_kd);
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity_X(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	/*
		if(1==Flag_Left)    	Movement=-Target_Velocity;	           //===前进标志位置1 
		else if(1==Flag_Right)	Movement=Target_Velocity;           //===后退标志位置1
  	else*/  
	Movement=0;
    //=============速度PI控制器=======================//	
		Encoder_Least=Mean_Filter_X(velocity);        //速度滤波  
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
 		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                      //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;               //===积分限幅
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===积分限幅	
	  //if(Flag_Stop)   Encoder_Integral=0; //===电机关闭后清除积分
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;        //===速度控制	
	  //if(Flag_Stop)   Velocity=0;      //===电机关闭后清除积分
		if(Velocity>1000)  	Velocity=1000;               //===速度环限幅
		if(Velocity<-1000)	  Velocity=-1000;              //===速度环限幅
	  return Velocity;
}
/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity_Y(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	/*
	  if(1==Flag_Qian)    	  Movement=Target_Velocity;	           //===前进标志位置1 
		else if(1==Flag_Hou)	  Movement=-Target_Velocity;           //===后退标志位置1
  	else
  */  
	Movement=0;
//   //=============速度PI控制器=======================//	
		Encoder_Least=Mean_Filter_Y(velocity);          //速度滤波      
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                  //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;            //===积分限幅
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===积分限幅	
    //if(Flag_Stop)   Encoder_Integral=0;      //===电机关闭后清除积分
  	Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;      //===速度控制	
	  //if(Flag_Stop)   Velocity=0;      //===电机关闭后清除积分
	  if(Velocity>1000)  	Velocity=1000;               //===速度环限幅
		if(Velocity<-1000)	  Velocity=-1000;              //===速度环限幅
	  return Velocity;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C;
			Final_Motor_A=Linear_Conversion(motor_a);  //线性化
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
	    if(motor_a<0) Final_Motor_A=-Final_Motor_A;
		  if(motor_b<0) Final_Motor_B=-Final_Motor_B;
	    if(motor_c<0) Final_Motor_C=-Final_Motor_C;

			set_motor(Final_Motor_A,Final_Motor_B,Final_Motor_C);  
}
/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/my_abs(motor);   //1000000是经验值
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
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
