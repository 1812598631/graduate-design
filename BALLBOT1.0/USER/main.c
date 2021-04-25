#include "pwm.h"
#include "delay.h"
#include "sys.h"
#include "motor.h"
#include "led.h"
#include "usart.h"	  
#include "control.h"
u8 Way_Angle=2;      //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 （有的6050使用DMP时，需要开机后不停摇晃小车10S左右，等待数据稳定）

int Motor_A,Motor_B,Motor_C;        //电机PWM变量
int Target_A,Target_B,Target_C;     //电机目标值
int compute_X,compute_Y,compute_Z;//正运动学解算三轴速度
int Angle_Zero_X, Angle_Zero_Y,Angle_Zero_Z,Angle_Bias_X, Angle_Bias_Y,Angle_Bias_Z;
float Angle_Balance_X,Angle_Balance_Y,Angle_Balance_Z,Gyro_Balance_X,Gyro_Balance_Z,Gyro_Balance_Y,Move_X,Move_Y,Move_Z;   //三轴角度和XYZ轴目标速度
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y,Balance_Pwm_Z;
float Angle_Balance_X_last,Angle_Balance_Y_last,Angle_Balance_Z_last,Gyro_Balance_X_last,Gyro_Balance_Y_last,Gyro_Balance_Z_last;
float bias_X,bias_Y,bias_Z,bias_x,bias_y,bias_z;
float	Balance_Kp=3,Balance_Kd=1 ,Velocity_Kp=0.5,Velocity_Ki=0,Turn_Kp=10,Turn_Kd=5;  //控制PID参数
u8 state_flag=0;


void scope(void);
int main(void)
{
u8 i=0;
delay_init(); 
Motor_Init();
LED_Init();		  	 	//初始化与LED连接的硬件接口
uart_init(115200);
TIM1_PWM_Init(71,71);
TIM3_PWM_Init(71,71);
TIM4_PWM_Init(71,71);//不分频。PWM频率=72000/(899+1)=80Khz 
set_motorA_speed(1,0);
set_motorB_speed(1,0);
set_motorC_speed(1,0);
	IIC_Init();                     //模拟IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //初始化DMP     

delay_ms(10);

while(1)
{
//	LED0=!LED0;
//  delay_ms(500);
//			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//获取Y方向的偏差
//		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//获取Y方向的偏差
// 			Encoder_Analysis(Motor_A,Motor_B,Motor_C);//对编码器的数据进行正运动学分析
	    if(state_flag==0)
			{
				
			  delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
        delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				
				state_flag=1;
				for(i=0;i<255;i++)
				Read_DMP(); 
        Angle_Zero_X=Angle_Balance_X;
				Angle_Zero_Y=Angle_Balance_Y;
				Angle_Zero_Z=Angle_Balance_Z;


			}
			/*
			printf("Roll:%f  ",Angle_Balance_X);  //X 
			printf("Pitch:%f  ",Angle_Balance_Y); //Y
 			printf("Yaw:%f\r\n",Angle_Balance_Z);   //Z
			*/
//			printf("Gyro_Balance_X:%f  ",Gyro_Balance_X);  //X 
//			printf("Gyro_Balance_Y:%f  ",Gyro_Balance_Y); //Y
// 			printf("Gyro_Balance_Z:%f\r\n",Gyro_Balance_Z);   //Z
						
			Read_DMP();                      //===读取倾角
      scope();
			Angle_Bias_X =Angle_Zero_X-Angle_Balance_X;		//获取Y方向的偏差
		  Angle_Bias_Y =Angle_Zero_Y-Angle_Balance_Y;		//获取Y方向的偏差
		  //Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
			Angle_Bias_Z =0;
	    Balance_Pwm_X= -balance_X(Angle_Bias_X,Gyro_Balance_X);//X方向的倾角控制
			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y方向的倾角控制
			Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z方向倾角控制
			
 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
			printf("Balance_Pwm_Z:%d  ",Balance_Pwm_Z); //Y
 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			
			
			printf("bias_x:%d  ",Angle_Bias_X);  //X 
			printf("bias_y:%d  ",Angle_Bias_Y);  //X 
 			printf("bias_z:%d\r\n",Angle_Bias_Z);   //Z
			
			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X方向控制量累加					
			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y方向控制量累加					
			Move_Z=Balance_Pwm_Z;				 //===Z方向控制量累加	
 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//逆运动学分析，得到A B C电机控制量
			Motor_A=Target_A;//直接调节PWM占空比 
			Motor_B=Target_B;//直接调节PWM占空比
			Motor_C=Target_C;//直接调节PWM占空比
			
 			printf("Target_A:%d  ",Target_A);  //X 
			printf("Target_B:%d  ",Target_B);  //X 
			printf("Target_C:%d \r\n ",Target_C); //Y
			
			set_motor(Motor_A,Motor_B,Motor_C);
			Gyro_Balance_X_last=Gyro_Balance_X;
			Gyro_Balance_Y_last=Gyro_Balance_Y;
			Gyro_Balance_Z_last=Gyro_Balance_Z;
			Angle_Balance_X_last=Angle_Balance_X;
			Angle_Balance_Y_last=Angle_Balance_Y;
			Angle_Balance_Z_last=Angle_Balance_Z;
}
}
void scope(void)
{
			bias_x=Gyro_Balance_X_last-Gyro_Balance_X;
			bias_y=Gyro_Balance_Y_last-Gyro_Balance_Y;
			bias_z=Gyro_Balance_Z_last-Gyro_Balance_Z;
//			printf("bias_x:%f",bias_x);  //X 
//			printf("bias_Y:%f",bias_y);  //X 
//			printf("bias_Z:%f\r\n",bias_z);  //X 
			bias_X=Angle_Balance_X_last-Angle_Balance_X;
			bias_Y=Angle_Balance_Y_last-Angle_Balance_Y;
			bias_Z=Angle_Balance_Z_last-Angle_Balance_Z;

			if(bias_x>1000||bias_x<-1000) Gyro_Balance_X=Gyro_Balance_X_last;
			if(bias_y>1000||bias_y<-1000) Gyro_Balance_Y=Gyro_Balance_Y_last;
			if(bias_z>5000||bias_z<-5000) Gyro_Balance_Z=Gyro_Balance_Z_last;

			if(bias_X>1000||bias_X<-1000) Angle_Balance_X=Angle_Balance_X_last;
			if(bias_Y>1000||bias_Y<-1000) Angle_Balance_Y=Angle_Balance_Y_last;
			if(bias_Z>5000||bias_Z<-5000) Angle_Balance_Z=Angle_Balance_Z_last;
}

