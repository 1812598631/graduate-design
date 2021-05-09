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




void Tim1_Init(int arr,int psc)//5ms进入一次
{ 
   TIM_TimeBaseInitTypeDef TIM_Structure;               //定义定时器结构体变量
	 NVIC_InitTypeDef NVIC_TIM;                           //定义中断嵌套结构体变量
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  //打开定时器时钟
	 
	 TIM_Structure.TIM_Period = (arr-1) ;         //设置自动重装载寄存器周期值  溢出时间TimeOut= (arr)*(psc)/Tic    单位为us
   TIM_Structure.TIM_Prescaler = (psc-1);       //设置预分频值     
   TIM_Structure.TIM_CounterMode = TIM_CounterMode_Up ;     //计数模式 上升计数
	 TIM_Structure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频      Tic=72M/（TIM_ClockDivision+1）
	 TIM_Structure.TIM_RepetitionCounter = 0; //重复计数的次数
	
	 TIM_TimeBaseInit(TIM1,&TIM_Structure);   //初始化定时器1
	
	 NVIC_TIM.NVIC_IRQChannel = TIM1_UP_IRQn;  //定时器1的向上计算通道
	 NVIC_TIM.NVIC_IRQChannelCmd = ENABLE ;    //使能
	 NVIC_TIM.NVIC_IRQChannelPreemptionPriority = 0 ;    //抢占优先级
	 NVIC_TIM.NVIC_IRQChannelSubPriority = 0;            //响应优先级 
	
	 NVIC_Init(&NVIC_TIM);                     //初始化结构体
	 
	 TIM_ClearFlag(TIM1,TIM_FLAG_Update);      //清空所有标志位  保证工作状态初始化 
	 
	 TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  //打开计时器
	
	 TIM_Cmd(TIM1,ENABLE);      	  	 	       //打开TIM1
	
	 
}	

/*****定时器1中断响应函数*****/
/*   需要查询中断函数名字 请打开 startup_stm32f10x_hd.s启动文件查询对应的中断响应名字*/
/*   说明：TIM_ClearFlag()函数是清除该定时器的所有标志位  
          一个定时器的标志位包括很多如 TIM_IT_Update TIM_IT_CC1 TIM_IT_CC2 TIM_IT_CC3 等 
          所以我们用具体清空标志位函数TIM_ClearITPendingBit() 对某个标志位进行清除 如果在没有几个标志位同时工作 两函数的使用效果应该是一样的  */

void TIM1_UP_IRQHandler (void)                
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update) != RESET)   //如果中断标志被置1 证明有中断
	{
	 
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);    // 清空标志位，为下一次进入中断做准备
			Angle_Bias_X =Angle_Balance_X-Angle_Zero_X;		//获取Y方向的偏差
		  Angle_Bias_Y =Angle_Balance_Y-Angle_Zero_Y;		//获取Y方向的偏差
		  //Angle_Bias_Z =Angle_Zero_Z-Angle_Balance_Z;
			//Angle_Bias_Z =0;
			//Encoder_Analysis(Motor_A,Motor_B,Motor_C);  //正运动学分析，得到X Y Z 方向的速度
//	    printf("compute_X:%d  ",compute_X);  //X 
//			printf("compute_Y:%d  ",compute_Y); //Y
// 			printf("compute_Z:%d\r\n",compute_Z);   //Z
			Balance_Pwm_X= balance_X(Angle_Bias_X,Gyro_Balance_X);//X方向的倾角控制
			Balance_Pwm_Y= -balance_Y(Angle_Bias_Y,Gyro_Balance_Y);	//Y方向的倾角控制
			//Balance_Pwm_Z= -balance_Z(Angle_Bias_Z,Gyro_Balance_Z);		//Z方向倾角控制
			//Velocity_Pwm_X=velocity_X(compute_X);      //X方向的速度控制
			//Velocity_Pwm_Y=velocity_Y(compute_Y);  	  //Y方向的速度控制  
			
// 			printf("Balance_Pwm_X:%d  ",Balance_Pwm_X);  //X 
//			printf("Balance_Pwm_Y:%d  ",Balance_Pwm_Y);  //X 
//			printf("Balance_Pwm_Z:%d\r\n  ",Balance_Pwm_Z); //Y
// 			printf("Angle_Bias_Z:%f\r\n",Angle_Balance_Z);   //Z
			

			
			Move_X =Balance_Pwm_X+Velocity_Pwm_X;   //===X方向控制量累加					
			Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;   //===Y方向控制量累加					
			Move_Z=0;				 //===Z方向控制量累加	
 		  Kinematic_Analysis(Move_X,Move_Y,Move_Z);//逆运动学分析，得到A B C电机控制量
			Motor_A=Target_A;//直接调节PWM占空比 
			Motor_B=Target_B;//直接调节PWM占空比
			Motor_C=Target_C;//直接调节PWM占空比
			Gyro_Balance_X_last=Gyro_Balance_X;
			Gyro_Balance_Y_last=Gyro_Balance_Y;
			Gyro_Balance_Z_last=Gyro_Balance_Z;
			Angle_Balance_X_last=Angle_Balance_X;
			Angle_Balance_Y_last=Angle_Balance_Y;
			Angle_Balance_Z_last=Angle_Balance_Z;
	}
  
}


//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
 
void TIM2_PWM_Init(u16 arr,u16 psc)
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
