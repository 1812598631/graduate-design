#include "pwm.h"
#include "delay.h"
#include "sys.h"
#include "motor.h"
#include "led.h"
#include "usart.h"	  
#include "control.h"
u8 Way_Angle=2;      //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� ���е�6050ʹ��DMPʱ����Ҫ������ͣҡ��С��10S���ң��ȴ������ȶ���

int Motor_A,Motor_B,Motor_C;        //���PWM����
int Target_A,Target_B,Target_C;     //���Ŀ��ֵ
int compute_X,compute_Y,compute_Z;//���˶�ѧ���������ٶ�
int Angle_Zero_X, Angle_Zero_Y,Angle_Zero_Z,Angle_Bias_X, Angle_Bias_Y,Angle_Bias_Z;
float Angle_Balance_X,Angle_Balance_Y,Angle_Balance_Z,Gyro_Balance_X,Gyro_Balance_Z,Gyro_Balance_Y,Move_X,Move_Y,Move_Z;   //����ǶȺ�XYZ��Ŀ���ٶ�
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y,Balance_Pwm_Z;
float Angle_Balance_X_last,Angle_Balance_Y_last,Angle_Balance_Z_last,Gyro_Balance_X_last,Gyro_Balance_Y_last,Gyro_Balance_Z_last;
float bias_X,bias_Y,bias_Z,bias_x,bias_y,bias_z;
float	Balance_Kp=3,Balance_Kd=1 ,Velocity_Kp=0.5,Velocity_Ki=0,Turn_Kp=10,Turn_Kd=5;  //����PID����
u8 state_flag=0;


void scope(void);
int main(void)
{
u8 i=0;
delay_init(); 
Motor_Init();
LED_Init();		  	 	//��ʼ����LED���ӵ�Ӳ���ӿ�
uart_init(115200);
TIM1_PWM_Init(71,71);
TIM3_PWM_Init(71,71);
TIM4_PWM_Init(71,71);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 
set_motorA_speed(1,0);
set_motorB_speed(1,0);
set_motorC_speed(1,0);
	IIC_Init();                     //ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //��ʼ��DMP     

delay_ms(10);

while(1)
{
//	LED0=!LED0;
//  delay_ms(500);
//			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//��ȡY�����ƫ��
//		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//��ȡY�����ƫ��
// 			Encoder_Analysis(Motor_A,Motor_B,Motor_C);//�Ա����������ݽ������˶�ѧ����
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
						
			Read_DMP();                      //===��ȡ���
      scope();
			Angle_Bias_X =Angle_Zero_X-Angle_Balance_X;		//��ȡY�����ƫ��
		  Angle_Bias_Y =Angle_Zero_Y-Angle_Balance_Y;		//��ȡY�����ƫ��
		  //Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
			Angle_Bias_Z =0;
	    Balance_Pwm_X= -balance_X(Angle_Bias_X,Gyro_Balance_X);//X�������ǿ���
			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y�������ǿ���
			Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z������ǿ���
			
 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
			printf("Balance_Pwm_Z:%d  ",Balance_Pwm_Z); //Y
 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			
			
			printf("bias_x:%d  ",Angle_Bias_X);  //X 
			printf("bias_y:%d  ",Angle_Bias_Y);  //X 
 			printf("bias_z:%d\r\n",Angle_Bias_Z);   //Z
			
			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X����������ۼ�					
			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y����������ۼ�					
			Move_Z=Balance_Pwm_Z;				 //===Z����������ۼ�	
 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//���˶�ѧ�������õ�A B C���������
			Motor_A=Target_A;//ֱ�ӵ���PWMռ�ձ� 
			Motor_B=Target_B;//ֱ�ӵ���PWMռ�ձ�
			Motor_C=Target_C;//ֱ�ӵ���PWMռ�ձ�
			
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

