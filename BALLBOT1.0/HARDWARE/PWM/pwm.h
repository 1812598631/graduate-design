#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

void Tim1_Init(int arr,int psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);

#endif
