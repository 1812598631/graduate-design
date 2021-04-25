#include "motor.h"


  /**************************************************************************
���ߣ�THE RUN
�ҵ�GITHUB��ҳ��
�������ת�ٹ�ʽ�����ٶ�w=(f*60)/(200*sub_num)
32��ʱ������:T=(pre+1)*(arr+1)/72M   APB2:72M APB1:36M
T=1/f Ĭ�� sub_num=1��arr+1=72 
��w=0.3f T=1/f=(pre+1)/1M


**************************************************************************/

u16 sub_num=1;
void Motor_Init(void) 
{
GPIO_InitTypeDef GPIO_InitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA,ENABLE);            //GPIO CLOCK ENABLE  
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;  
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_SetBits(GPIOB,GPIO_Pin_1);
GPIO_SetBits(GPIOA,GPIO_Pin_1);
GPIO_SetBits(GPIOA,GPIO_Pin_2);
GPIO_SetBits(GPIOA,GPIO_Pin_3);
}
u32 get_CCR(u8 clk,u16 speed)//timer1 clk=72 timer 3,timer 4 clk=36
{
  u32 fre;
	u16 arr=71;

	double pre;
	double k;
	k=pow(10,6);
	fre=(speed*200*sub_num)/60;
	if(fre==0)
	pre=0;
  else
  pre=(clk*k)/(fre*(arr+1));
	return pre;
}
//period ���� ���� ������Ҫ���� 
void Step_Control(u8 dir,u16 period,u32 steps) //λ�ÿ���
{
  u32 i,half_period; 
	half_period=period/2;
if(dir==0)
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
else
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
for(i=0; i < steps;i++) 
{
GPIO_SetBits(GPIOB,GPIO_Pin_1);
delay_us(half_period); 
GPIO_ResetBits(GPIOB,GPIO_Pin_1);
delay_us(half_period); 
}
}

void set_motorA_speed(u8 dir,u16 speed)
{
	u32 arr;
	arr=get_CCR(36,speed);
	/*
  TIM_PrescalerConfig(TIM3,CCR1_Val,TIM_PSCReloadMode_Immediate);
  TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3,ENABLE);
	*/
		TIM_ARRPreloadConfig(TIM3,DISABLE);
		
		TIM3->ARR=arr;//������10000�ڹ������¼���
		TIM3->CCR4=arr/2;//����ռ�ձ�Ϊ50%
		TIM_ARRPreloadConfig(TIM3,ENABLE);
		TIM_Cmd(TIM3,ENABLE);
		  USART_SendData(USART1, 0X0D);
	  USART_SendData(USART1, 0X0A);	
	if(dir==0)
	{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	}
  else
  {
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
  }
}

void set_motorB_speed(u8 dir,u16 speed)
{
	u32 arr;
	arr=get_CCR(36,speed);
		TIM_ARRPreloadConfig(TIM4,DISABLE);
		
		TIM4->ARR=arr;//������10000�ڹ������¼���
		TIM4->CCR4=arr/2;//����ռ�ձ�Ϊ50%
		TIM_ARRPreloadConfig(TIM4,ENABLE);
		TIM_Cmd(TIM4,ENABLE);
	  USART_SendData(USART1, 0X0D);
	  USART_SendData(USART1, 0X0A);	 //�س�
	if(dir==0)
	{
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
	}
  else 
  {
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
  }
}
void set_motorC_speed(u8 dir,u16 speed)
{
	u32 arr;
	arr=get_CCR(36,speed);
		TIM_ARRPreloadConfig(TIM2,DISABLE);
		
		TIM2->ARR=arr;//������10000�ڹ������¼���
		TIM2->CCR1=arr/2;//����ռ�ձ�Ϊ50%
		TIM_ARRPreloadConfig(TIM2,ENABLE);
		TIM_Cmd(TIM2,ENABLE);
		//printf("arr_c=%d",arr);

	if(dir==0)
	{
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
	}
  else
  {
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
  }
}
void set_motor(int motor_a,int motor_b,int motor_c)
{
    	if(motor_a>0)			set_motorA_speed(0,motor_a);//����BTN7971оƬд�����߼�
			else  	          set_motorA_speed(1,-motor_a);//����BTN7971оƬд�����߼�
		
		  if(motor_b>0)			set_motorB_speed(0,motor_b);
			else 	            set_motorB_speed(1,-motor_b);
	
	    if(motor_c>0)			set_motorC_speed(0,motor_c);
			else 	            set_motorC_speed(1,-motor_c);
}

