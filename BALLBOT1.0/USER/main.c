#include "pwm.h"
#include "delay.h"
#include "sys.h"
#include "motor.h"
#include "led.h"
#include "usart.h"	  
#include "control.h"
#include "OLED_I2C.h"

u8 Way_Angle=2;      //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
//u8 i=0;
int Motor_A,Motor_B,Motor_C;        //电机PWM变量
int Target_A,Target_B,Target_C;     //电机目标值
int compute_X,compute_Y,compute_Z;//正运动学解算三轴速度
float Angle_Zero_X, Angle_Zero_Y,Angle_Zero_Z,Angle_Bias_X, Angle_Bias_Y,Angle_Bias_Z;
float Angle_Balance_X,Angle_Balance_Y,Angle_Balance_Z,Gyro_Balance_X,Gyro_Balance_Z,Gyro_Balance_Y,Move_X,Move_Y,Move_Z;   //三轴角度和XYZ轴目标速度
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y,Balance_Pwm_Z;
float Angle_Balance_X_last,Angle_Balance_Y_last,Angle_Balance_Z_last,Gyro_Balance_X_last,Gyro_Balance_Y_last,Gyro_Balance_Z_last;
float bias_X,bias_Y,bias_Z,bias_x,bias_y,bias_z;
float	Balance_Kp=40,Balance_Kd=15,Velocity_Kp=0.5,Velocity_Ki=0.01,Turn_Kp=10,Turn_Kd=0;  //控制PID参数
u8 state_flag=0;
u8 time_flag=0;
unsigned char init[13]="BALLBOT init"; //字符串Distance
unsigned char correct[16]="self-correcting"; //字符串Distance

unsigned char x[3]="x:"; //字符串Distance
unsigned char y[3]="y:"; //字符串Distance
unsigned char tarA[3]="A:"; //字符串Distance
unsigned char tarB[3]="B:"; //字符串Distance
unsigned char tarC[3]="C:"; //字符串Distance

void scope(void);
u32 myabs(int a);
void wait_err(void);
void set_max(int max);
int main(void)
{
u8 i=0;
delay_init(); 
Motor_Init();
LED_Init();		  	 	//初始化与LED连接的硬件接口
uart_init(115200);
Tim1_Init(36,1000);//0.5ms进入一次                                                                 
TIM2_PWM_Init(71,71);
TIM3_PWM_Init(71,71);
TIM4_PWM_Init(71,71);
set_motorA_speed(1,0);
set_motorB_speed(1,0);
set_motorC_speed(1,0);
IIC_Init();                     //模拟IIC初始化
MPU6050_initialize();           //=====MPU6050初始化	
DMP_Init();                     //初始化DMP     
I2C_Configuration();
OLED_Init();													//OLED初始化
OLED_CLS();		 //清屏
delay_ms(10);

while(1)
{
	    if(state_flag==0)
			{
				OLED_ShowStr(0,1,init,2);		  	//字符串2，必须放在数据后面，否则会乱码
				OLED_ShowStr(0,3,correct,1);		  	//字符串2，必须放在数据后面，否则会乱码
				//wait_err();
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
				OLED_CLS();		 //清屏
			}
			
					LED0=!LED0;
						Read_DMP();                      //===读取倾角
						scope();
				//	delay_ms(50);
			/*
			printf("Roll:%f  ",Angle_Balance_X);  //X 
			printf("Pitch:%f  ",Angle_Balance_Y); //Y
 			printf("Yaw:%f\r\n",Angle_Balance_Z);   //Z
			
			printf("Gyro_Balance_X:%f  ",Gyro_Balance_X);  //X 
			printf("Gyro_Balance_Y:%f  ",Gyro_Balance_Y); //Y
 			printf("Gyro_Balance_Z:%f\r\n",Gyro_Balance_Z);   //Z
      */
			
//			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//获取Y方向的偏差
//		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//获取Y方向的偏差
//      Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
//			Angle_Bias_Z =0;
			//Encoder_Analysis(Motor_A,Motor_B,Motor_C);  //正运动学分析，得到X Y Z 方向的速度
//	    printf("compute_X:%d  ",compute_X);  //X 
//			printf("compute_Y:%d  ",compute_Y); //Y
// 			printf("compute_Z:%d\r\n",compute_Z);   //Z
//			Balance_Pwm_X= balance_X(Angle_Bias_X,Gyro_Balance_X);//X方向的倾角控制
//			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y方向的倾角控制
//			Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z方向倾角控制
			//Velocity_Pwm_X=velocity_X(compute_X);      //X方向的速度控制
			//Velocity_Pwm_Y=velocity_Y(compute_Y);  	  //Y方向的速度控制  
			
// 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
//			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
//			printf("Balance_Pwm_Z:%d\r\n  ",Balance_Pwm_Z); //Y
// 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			

			
//			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X方向控制量累加					
//			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y方向控制量累加					
//			Move_Z=0;				 //===Z方向控制量累加	
// 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//逆运动学分析，得到A B C电机控制量
//			Motor_A=Target_A;//直接调节PWM占空比 
//			Motor_B=Target_B;//直接调节PWM占空比
//			Motor_C=Target_C;//直接调节PWM占空比
		  /*
			OLED_ShowStr(0,1,x,1);		  	//字符串2，必须放在数据后面，否则会乱码
		  Oled_write_data_3(17,1,Angle_Bias_X);		 //数据显示
			OLED_ShowStr(0,2,y,1);		  	//字符串2，必须放在数据后面，否则会乱码
		  Oled_write_data_3(17,2,Angle_Bias_Y);		 //数据显示
			OLED_ShowStr(0,3,tarA,1);		  	//字符串2，必须放在数据后面，否则会乱码
		  Oled_write_data_3(17,3,Target_A);		 //数据显示
			OLED_ShowStr(0,4,tarB,1);		  	//字符串2，必须放在数据后面，否则会乱码
		  Oled_write_data_3(17,4,Target_B);		 //数据显示
			OLED_ShowStr(0,5,tarC,1);		  	//字符串2，必须放在数据后面，否则会乱码
		  Oled_write_data_3(17,5,Target_C);		 //数据显示
			*/
			
//			printf("bias_x:%.1f  ",Angle_Bias_X);  //X 
//			printf("bias_y:%.1f  ",Angle_Bias_Y);  //X 
// 			printf("bias_z:%.1f\r\n",Angle_Bias_Z);   //Z
			  //set_max(2000);
				Xianfu_Pwm(600);
				Set_Pwm(Motor_A,Motor_B,Motor_C);
//				printf("Target_A:%d  ",Motor_A);  //X 
//			  printf("Target_B:%d  ",Motor_B);  //X 
//			  printf("Target_C:%d \r\n ",Motor_C); //Y
//			Gyro_Balance_X_last=Gyro_Balance_X;
//			Gyro_Balance_Y_last=Gyro_Balance_Y;
//			Gyro_Balance_Z_last=Gyro_Balance_Z;
//			Angle_Balance_X_last=Angle_Balance_X;
//			Angle_Balance_Y_last=Angle_Balance_Y;
//			Angle_Balance_Z_last=Angle_Balance_Z;
//	printf("Gyro_last_X:%.1f  ",Gyro_Balance_X_last); //Y
//	printf("Gyro_last_Y:%.1f\r\n  ",Gyro_Balance_Y_last); //Y
			
}
}
void wait_err(void)
{
	u8 i=0;
float y_err=1,x_err=1,z_err=1,a_x=1,a_y=1,a_z=1,a_x_l=1,a_y_l=1,a_z_l=1;
while((x_err>0.2||x_err<-0.2)&&(y_err>0.2||y_err<-0.2)&&(z_err>0.2||z_err<-0.2))
{

				for(i=0;i<255;i++)
				Read_DMP();     
	a_x=Angle_Balance_X;
	a_y=Angle_Balance_Y;
	a_z=Angle_Balance_Z;	
	x_err=a_x-a_x_l;
	y_err=a_y-a_y_l;
	z_err=a_z-a_z_l;
	printf("a_x:%f  ",a_x); //Y
	printf("a_y:%f  ",a_y); //Y
	printf("a_z:%f  ",a_z); //Y
	printf("a_x_l:%f  ",a_x_l); //Y
	printf("a_y_l:%f  ",a_y_l); //Y
 	printf("a_z_l:%f\r\n",a_z_l);   //Z
	a_x_l=Angle_Balance_X;
	a_y_l=Angle_Balance_Y;
	a_z_l=Angle_Balance_Z;
	delay_ms(1000);
}
}
void set_max(int max)
{
if(Motor_A>max) Motor_A=max; else if(Motor_A<-max) Motor_A=-max;
if(Motor_B>max) Motor_B=max; else if(Motor_B<-max) Motor_B=-max;
if(Motor_C>max) Motor_C=max; else if(Motor_C<-max) Motor_C=-max;
}
void scope(void)
{
			bias_x=Gyro_Balance_X_last-Gyro_Balance_X;
			bias_y=Gyro_Balance_Y_last-Gyro_Balance_Y;
			bias_z=Gyro_Balance_Z_last-Gyro_Balance_Z;

	    bias_X=Angle_Balance_X_last-Angle_Balance_X;
			bias_Y=Angle_Balance_Y_last-Angle_Balance_Y;
			bias_Z=Angle_Balance_Z_last-Angle_Balance_Z;
//	printf("bias_x:%.1f  ",bias_x); //Y
//	printf("bias_y:%.1f  ",bias_y); //Y
// 	printf("bias_z:%.1f\r\n",bias_z);   //Z
	    bias_x=myabs(bias_x);
		  bias_y=myabs(bias_y);
	    bias_z=myabs(bias_z);
			if(bias_x>8000||bias_y>8000||bias_z>8000) 
			{
				Gyro_Balance_X=Gyro_Balance_X_last;
				Gyro_Balance_Y=Gyro_Balance_Y_last;
				Gyro_Balance_Z=Gyro_Balance_Z_last;
			}

			if(bias_X>1000||bias_X<-1000) Angle_Balance_X=Angle_Balance_X_last;
			if(bias_Y>1000||bias_Y<-1000) Angle_Balance_Y=Angle_Balance_Y_last;
			if(bias_Z>5000||bias_Z<-5000) Angle_Balance_Z=Angle_Balance_Z_last;
//				printf("ANGLE_X:%.1f  ",Angle_Balance_X); //Y
//	      printf("ANGLE_Y:%.1f\r\n  ",Angle_Balance_Y); //Y
//				printf("bias_X:%.1f  ",bias_X); //Y
//	      printf("bias_Y:%.1f\r\n  ",bias_Y); //Y
//			
//				printf("compute_X:%.1f  ",Gyro_Balance_X); //Y
//	printf("compute_Y:%.1f\r\n  ",Gyro_Balance_Y); //Y
}


u32 myabs( int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
