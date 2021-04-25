#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "pwm.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/;
#define STEP_PER_REV 200*sub_num;

void Motor_Init(void) ;
void Step_Control(u8 dir,u16 period,u32 steps) ;
u32 get_CCR(u8 clk,u16 speed);
void set_motorA_speed(u8 dir,u16 speed);
void set_motorB_speed(u8 dir,u16 speed);
void set_motorC_speed(u8 dir,u16 speed);
void set_motor(int motor_a,int motor_b,int motor_c);
#endif
