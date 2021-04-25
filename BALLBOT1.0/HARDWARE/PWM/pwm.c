#include "pwm.h"
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
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
 
void TIM1_PWM_Init(u16 arr,u16 psc)
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
