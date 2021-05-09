#include "pwm.h"
#include "delay.h"
#include "sys.h"
#include "motor.h"
#include "led.h"
#include "usart.h"	  
#include "control.h"
#include "OLED_I2C.h"

u8 Way_Angle=2;      //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
//u8 i=0;
int Motor_A,Motor_B,Motor_C;        //���PWM����
int Target_A,Target_B,Target_C;     //���Ŀ��ֵ
int compute_X,compute_Y,compute_Z;//���˶�ѧ���������ٶ�
float Angle_Zero_X, Angle_Zero_Y,Angle_Zero_Z,Angle_Bias_X, Angle_Bias_Y,Angle_Bias_Z;
float Angle_Balance_X,Angle_Balance_Y,Angle_Balance_Z,Gyro_Balance_X,Gyro_Balance_Z,Gyro_Balance_Y,Move_X,Move_Y,Move_Z;   //����ǶȺ�XYZ��Ŀ���ٶ�
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y,Balance_Pwm_Z;
float Angle_Balance_X_last,Angle_Balance_Y_last,Angle_Balance_Z_last,Gyro_Balance_X_last,Gyro_Balance_Y_last,Gyro_Balance_Z_last;
float bias_X,bias_Y,bias_Z,bias_x,bias_y,bias_z;
float	Balance_Kp=40,Balance_Kd=15,Velocity_Kp=0.5,Velocity_Ki=0.01,Turn_Kp=10,Turn_Kd=0;  //����PID����
u8 state_flag=0;
u8 time_flag=0;
unsigned char init[13]="BALLBOT init"; //�ַ���Distance
unsigned char correct[16]="self-correcting"; //�ַ���Distance

unsigned char x[3]="x:"; //�ַ���Distance
unsigned char y[3]="y:"; //�ַ���Distance
unsigned char tarA[3]="A:"; //�ַ���Distance
unsigned char tarB[3]="B:"; //�ַ���Distance
unsigned char tarC[3]="C:"; //�ַ���Distance

void scope(void);
u32 myabs(int a);
void wait_err(void);
void set_max(int max);
int main(void)
{
u8 i=0;
delay_init(); 
Motor_Init();
LED_Init();		  	 	//��ʼ����LED���ӵ�Ӳ���ӿ�
uart_init(115200);
Tim1_Init(36,1000);//0.5ms����һ��                                                                 
TIM2_PWM_Init(71,71);
TIM3_PWM_Init(71,71);
TIM4_PWM_Init(71,71);
set_motorA_speed(1,0);
set_motorB_speed(1,0);
set_motorC_speed(1,0);
IIC_Init();                     //ģ��IIC��ʼ��
MPU6050_initialize();           //=====MPU6050��ʼ��	
DMP_Init();                     //��ʼ��DMP     
I2C_Configuration();
OLED_Init();													//OLED��ʼ��
OLED_CLS();		 //����
delay_ms(10);

while(1)
{
	    if(state_flag==0)
			{
				OLED_ShowStr(0,1,init,2);		  	//�ַ���2������������ݺ��棬���������
				OLED_ShowStr(0,3,correct,1);		  	//�ַ���2������������ݺ��棬���������
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
				OLED_CLS();		 //����
			}
			
					LED0=!LED0;
						Read_DMP();                      //===��ȡ���
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
			
//			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//��ȡY�����ƫ��
//		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//��ȡY�����ƫ��
//      Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
//			Angle_Bias_Z =0;
			//Encoder_Analysis(Motor_A,Motor_B,Motor_C);  //���˶�ѧ�������õ�X Y Z ������ٶ�
//	    printf("compute_X:%d  ",compute_X);  //X 
//			printf("compute_Y:%d  ",compute_Y); //Y
// 			printf("compute_Z:%d\r\n",compute_Z);   //Z
//			Balance_Pwm_X= balance_X(Angle_Bias_X,Gyro_Balance_X);//X�������ǿ���
//			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y�������ǿ���
//			Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z������ǿ���
			//Velocity_Pwm_X=velocity_X(compute_X);      //X������ٶȿ���
			//Velocity_Pwm_Y=velocity_Y(compute_Y);  	  //Y������ٶȿ���  
			
// 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
//			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
//			printf("Balance_Pwm_Z:%d\r\n  ",Balance_Pwm_Z); //Y
// 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			

			
//			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X����������ۼ�					
//			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y����������ۼ�					
//			Move_Z=0;				 //===Z����������ۼ�	
// 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//���˶�ѧ�������õ�A B C���������
//			Motor_A=Target_A;//ֱ�ӵ���PWMռ�ձ� 
//			Motor_B=Target_B;//ֱ�ӵ���PWMռ�ձ�
//			Motor_C=Target_C;//ֱ�ӵ���PWMռ�ձ�
		  /*
			OLED_ShowStr(0,1,x,1);		  	//�ַ���2������������ݺ��棬���������
		  Oled_write_data_3(17,1,Angle_Bias_X);		 //������ʾ
			OLED_ShowStr(0,2,y,1);		  	//�ַ���2������������ݺ��棬���������
		  Oled_write_data_3(17,2,Angle_Bias_Y);		 //������ʾ
			OLED_ShowStr(0,3,tarA,1);		  	//�ַ���2������������ݺ��棬���������
		  Oled_write_data_3(17,3,Target_A);		 //������ʾ
			OLED_ShowStr(0,4,tarB,1);		  	//�ַ���2������������ݺ��棬���������
		  Oled_write_data_3(17,4,Target_B);		 //������ʾ
			OLED_ShowStr(0,5,tarC,1);		  	//�ַ���2������������ݺ��棬���������
		  Oled_write_data_3(17,5,Target_C);		 //������ʾ
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
