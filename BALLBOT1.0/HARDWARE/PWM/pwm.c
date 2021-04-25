#include "pwm.h"
volatile u32 pulse_width1 = 0;
volatile u32	direction1 = 0;
volatile u32 pulse_width2 = 0;
volatile u32	direction2 = 0;
volatile u32 pulse_width3 = 0;
volatile u32	direction3 = 0;
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//PWM  驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

float duty1=0.5;
float duty2=0.5;
u16 capture=0;
u8 pa6_state=0,pa7_state=0;
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
 
void TIM1_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO口复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO口速度
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//USART输出IO口
	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //定时周期
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频1，36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频因子
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //输出PWM模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能输出
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
 
//可以选择有中断或无中断
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO口复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO口速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//USART输出IO口
	
	TIM_DeInit(TIM3);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //定时周期
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频1，36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频因子
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //输出PWM模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能输出
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
 
//可以选择有中断或无中断
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
//可以选择有中断或无中断
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                    //IO口复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //IO口速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//USART输出IO口
	
	TIM_DeInit(TIM4);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //定时周期
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频1，36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频因子
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //输出PWM模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能输出
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性
	TIM_OCInitStructure.TIM_Pulse = 50;
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
 
//可以选择有中断或无中断
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

//可以选择有中断或无中断
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
