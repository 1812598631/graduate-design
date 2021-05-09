#include "pwm.h"
#include "sys.h"
#include "control.h"

volatile u32 pulse_width1 = 0;
volatile u32	direction1 = 0;
volatile u32 pulse_width2 = 0;
volatile u32	direction2 = 0;
volatile u32 pulse_width3 = 0;
volatile u32	direction3 = 0;
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//PWM  ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/03
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

float duty1=0.5;
float duty2=0.5;
u16 capture=0;
u8 pa6_state=0,pa7_state=0;




void Tim1_Init(int arr,int psc)//5ms����һ��
{ 
   TIM_TimeBaseInitTypeDef TIM_Structure;               //���嶨ʱ���ṹ�����
	 NVIC_InitTypeDef NVIC_TIM;                           //�����ж�Ƕ�׽ṹ�����
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  //�򿪶�ʱ��ʱ��
	 
	 TIM_Structure.TIM_Period = (arr-1) ;         //�����Զ���װ�ؼĴ�������ֵ  ���ʱ��TimeOut= (arr)*(psc)/Tic    ��λΪus
   TIM_Structure.TIM_Prescaler = (psc-1);       //����Ԥ��Ƶֵ     
   TIM_Structure.TIM_CounterMode = TIM_CounterMode_Up ;     //����ģʽ ��������
	 TIM_Structure.TIM_ClockDivision = TIM_CKD_DIV1;     //ʱ�ӷ�Ƶ      Tic=72M/��TIM_ClockDivision+1��
	 TIM_Structure.TIM_RepetitionCounter = 0; //�ظ������Ĵ���
	
	 TIM_TimeBaseInit(TIM1,&TIM_Structure);   //��ʼ����ʱ��1
	
	 NVIC_TIM.NVIC_IRQChannel = TIM1_UP_IRQn;  //��ʱ��1�����ϼ���ͨ��
	 NVIC_TIM.NVIC_IRQChannelCmd = ENABLE ;    //ʹ��
	 NVIC_TIM.NVIC_IRQChannelPreemptionPriority = 0 ;    //��ռ���ȼ�
	 NVIC_TIM.NVIC_IRQChannelSubPriority = 0;            //��Ӧ���ȼ� 
	
	 NVIC_Init(&NVIC_TIM);                     //��ʼ���ṹ��
	 
	 TIM_ClearFlag(TIM1,TIM_FLAG_Update);      //������б�־λ  ��֤����״̬��ʼ�� 
	 
	 TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  //�򿪼�ʱ��
	
	 TIM_Cmd(TIM1,ENABLE);      	  	 	       //��TIM1
	
	 
}	

/*****��ʱ��1�ж���Ӧ����*****/
/*   ��Ҫ��ѯ�жϺ������� ��� startup_stm32f10x_hd.s�����ļ���ѯ��Ӧ���ж���Ӧ����*/
/*   ˵����TIM_ClearFlag()����������ö�ʱ�������б�־λ  
          һ����ʱ���ı�־λ�����ܶ��� TIM_IT_Update TIM_IT_CC1 TIM_IT_CC2 TIM_IT_CC3 �� 
          ���������þ�����ձ�־λ����TIM_ClearITPendingBit() ��ĳ����־λ������� �����û�м�����־λͬʱ���� ��������ʹ��Ч��Ӧ����һ����  */

void TIM1_UP_IRQHandler (void)                
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update) != RESET)   //����жϱ�־����1 ֤�����ж�
	{
	 
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);    // ��ձ�־λ��Ϊ��һ�ν����ж���׼��
			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//��ȡY�����ƫ��
		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//��ȡY�����ƫ��
		  //Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
			//Angle_Bias_Z =0;
			//Encoder_Analysis(Motor_A,Motor_B,Motor_C);  //���˶�ѧ�������õ�X Y Z ������ٶ�
//	    printf("compute_X:%d  ",compute_X);  //X 
//			printf("compute_Y:%d  ",compute_Y); //Y
// 			printf("compute_Z:%d\r\n",compute_Z);   //Z
			Balance_Pwm_X= balance_X(Angle_Bias_X,Gyro_Balance_X);//X�������ǿ���
			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y�������ǿ���
			//Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z������ǿ���
			//Velocity_Pwm_X=velocity_X(compute_X);      //X������ٶȿ���
			//Velocity_Pwm_Y=velocity_Y(compute_Y);  	  //Y������ٶȿ���  
			
// 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
//			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
//			printf("Balance_Pwm_Z:%d\r\n  ",Balance_Pwm_Z); //Y
// 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			

			
			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X����������ۼ�					
			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y����������ۼ�					
			Move_Z=0;				 //===Z����������ۼ�	
 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//���˶�ѧ�������õ�A B C���������
			Motor_A=Target_A;//ֱ�ӵ���PWMռ�ձ� 
			Motor_B=Target_B;//ֱ�ӵ���PWMռ�ձ�
			Motor_C=Target_C;//ֱ�ӵ���PWMռ�ձ�
			Gyro_Balance_X_last=Gyro_Balance_X;
			Gyro_Balance_Y_last=Gyro_Balance_Y;
			Gyro_Balance_Z_last=Gyro_Balance_Z;
			Angle_Balance_X_last=Angle_Balance_X;
			Angle_Balance_Y_last=Angle_Balance_Y;
			Angle_Balance_Z_last=Angle_Balance_Z;
	}
  
}


//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
 
void TIM2_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO�ڸ����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO���ٶ�
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//USART���IO��
	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //Ԥ��Ƶ1��36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷ�Ƶ����
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //���PWMģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //ʹ�����
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�������
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
 
//����ѡ�����жϻ����ж�
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
//		
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
 
 
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		/*
		if(pulse_width1 == 0)
			direction1 = 0;
		else if(pulse_width1 == 100)
			direction1 = 1;
 
		if(direction1 == 0)
			pulse_width1++;
		else
			pulse_width1--;
		TIM_SetCompare1(TIM2, pulse_width1);
		*/
	}
 
}

void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO�ڸ����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO���ٶ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//USART���IO��
	
	TIM_DeInit(TIM3);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //Ԥ��Ƶ1��36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷ�Ƶ����
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //���PWMģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //ʹ�����
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�������
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
 
//����ѡ�����жϻ����ж�
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );
//		
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
 
 
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
//����ѡ�����жϻ����ж�
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		/*
		if(pulse_width2 == 0)
			direction2 = 0;
		else if(pulse_width2 == 100)
			direction2 = 1;
 
		if(direction2 == 0)
			pulse_width2++;
		else
			pulse_width2--;
		TIM_SetCompare4(TIM3, pulse_width2);
		*/
	}
 
}

void TIM4_PWM_Init(u16 arr,u16 psc)
{  
 	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO�ڸ����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO���ٶ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//USART���IO��
	
	TIM_DeInit(TIM4);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��ʱ����
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //Ԥ��Ƶ1��36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷ�Ƶ����
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //���PWMģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //ʹ�����
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�������
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
 
//����ѡ�����жϻ����ж�
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );
//		
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
 
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
 }

//����ѡ�����жϻ����ж�
void TIM4_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		/*
		if(pulse_width3 == 0)
			direction3 = 0;
		else if(pulse_width3 == 100)
			direction3 = 1;
 
		if(direction3 == 0)
			pulse_width3++;
		else
			pulse_width3--;
		TIM_SetCompare4(TIM4, pulse_width3);
		*/
	}
}
