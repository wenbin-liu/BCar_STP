#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_it.h"
#include "system_stm32f10x.h"

#include <stdio.h>

#include "uart.h"
#include "mpu6050.h"
#include "delay.h"
#include "balance.h"
#include "filter.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_it.h"
#include "misc.h"
#include <math.h>

#define MIDANGLE -2

int DEAD_PWM = 100;

extern float BALANCE_KP;
extern float BALANCE_KD;
extern float SPEED_KP,SPEED_KI,SPEED_KD;
extern float DIRECTION_KP;
int ControlSpeed;
int ControlDir;
void tim_irq_init()
{
	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM4EN,ENABLE);
    
    TIM_TimeBaseInitTypeDef tim_str;
    TIM_TimeBaseStructInit(&tim_str);
	
		tim_str.TIM_Prescaler=71;
    tim_str.TIM_CounterMode=TIM_CounterMode_Up;
    tim_str.TIM_Period=(uint16_t) 4999;
    TIM_TimeBaseInit(TIM4,&tim_str);
    
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

    TIM_Cmd(TIM4,ENABLE);
    NVIC_SetPriority(TIM4_IRQn,10);
    NVIC_EnableIRQ(TIM4_IRQn);
 
}

void tim_pwm_init()
{
	  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN|RCC_APB2ENR_AFIOEN,ENABLE);
	
	  TIM_TimeBaseInitTypeDef tim_str;
    TIM_TimeBaseStructInit(&tim_str);
	
    tim_str.TIM_CounterMode=TIM_CounterMode_Up;
    tim_str.TIM_Period=(uint16_t) 719;
    TIM_TimeBaseInit(TIM2,&tim_str);
	
		TIM_OCInitTypeDef tim_oc;
    TIM_OCStructInit(&tim_oc);
		tim_oc.TIM_OCMode=TIM_OCMode_PWM1;
		tim_oc.TIM_OCPolarity=TIM_OCPolarity_High;
    tim_oc.TIM_OCIdleState=TIM_OutputState_Enable;
		TIM_OC1Init(TIM2,&tim_oc);
		TIM_OC2Init(TIM2,&tim_oc);
		TIM_OC3Init(TIM2,&tim_oc);
		TIM_OC4Init(TIM2,&tim_oc);
	
		TIM_Cmd(TIM2,ENABLE);
	
		GPIO_InitTypeDef gpio_str;
		gpio_str.GPIO_Mode=GPIO_Mode_AF_PP;
		gpio_str.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
		gpio_str.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&gpio_str);
		TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_3,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_4,TIM_CCx_Enable);
		
}

void tim_encoder_init(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN|RCC_APB2ENR_TIM1EN|RCC_APB2ENR_IOPAEN,ENABLE);
 // GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
  
  TIM_TimeBaseInitTypeDef tim_base_init;
  TIM_TimeBaseStructInit(&tim_base_init);
  TIM_TimeBaseInit(TIM3,&tim_base_init);

  TIM_TimeBaseInit(TIM1,&tim_base_init);

  // TIM_ICInitTypeDef timic_str;
  // TIM_ICStructInit(&timic_str);
  // TIM_ICInit(TIM3,&timic_str);
  TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
  TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
  TIM_Cmd(TIM3,ENABLE);
  TIM_Cmd(TIM1,ENABLE);
  
  GPIO_InitTypeDef gpio_str;
  gpio_str.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  gpio_str.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
  gpio_str.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_str);

  gpio_str.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
  GPIO_Init(GPIOA,&gpio_str);

}
void USART3_IRQHandler(void)
{
	char c=USART_ReceiveData(USART3);
  if(c=='q')
    SPEED_KP+=0.1;
  else if(c=='a')
    SPEED_KP-=0.1;			
  else if(c=='w')
    SPEED_KI+=0.1;
  else if(c=='s')
    SPEED_KI-=0.1;
  else if(c=='e')
		SPEED_KD+=0.01;
  else if(c=='d')
    SPEED_KD-=0.01;
  else if(c=='i')
	{
    ControlSpeed=180;
		ControlDir=0;
	}
  else if(c=='k')
	{
    ControlSpeed=-180;
		ControlDir=0;
	}
  else if(c=='j')
    ControlDir=180;
  else if(c=='l')
    ControlDir=-180;
	else if(c=='u')
	{
		ControlSpeed=250;
		ControlDir=180;
	}
	else if(c=='o')
	{
		ControlSpeed=250;
		ControlDir=-180;
	}
	else
	{
		ControlSpeed=0;
		ControlDir=0;
	}

}

void TIM4_IRQHandler(void)
{
	static uint32_t count;
	float angle_raw,gyro;
	static int16_t pwmSpd,pwmSpdOld,pwmBalance,pwmDir,pwmL,pwmR;
	static float pwmSpdSlice;
  int16_t spdL,spdR;
  getAngle(&angle_raw,&gyro);
  Kalman_Filter(angle_raw,gyro);
  if(count%10==0)
    {
      printf("%.2f %.2f %d %d %.1f %.1f %.1f\r\n",angle,angle_dot,ControlSpeed,ControlDir,SPEED_KP,SPEED_KI,SPEED_KD);
      spdR = (int16_t)TIM_GetCounter(TIM3);
      spdL= (int16_t)TIM_GetCounter(TIM1);
      TIM_SetCounter(TIM3,0);
      TIM_SetCounter(TIM1,0);
			pwmSpdOld=pwmSpd;
      pwmSpd=-speedControl(ControlSpeed,spdL,spdR,ControlSpeed);
			pwmDir=directionControl(ControlDir,spdR-spdL)*0.5+pwmDir*0.5;
			pwmSpdSlice=pwmSpdOld;

    }
//	if(count%50==0)
//	{
//					ControlSpeed=0;
//			ControlDir=0;
//	}
	pwmSpdSlice+=(pwmSpd-pwmSpdOld)/10;
  pwmBalance= balanceControl(MIDANGLE,angle,angle_dot);
	pwmL=pwmBalance+pwmSpdSlice-pwmDir;
	pwmR=pwmBalance+pwmSpdSlice+pwmDir;
	if(pwmL>0)
		pwmL+=DEAD_PWM;
	else if(pwmL<0)
		pwmL-=DEAD_PWM;
	
	if(pwmR>0)
		pwmR+=DEAD_PWM;
	else if(pwmR<0)
		pwmR-=DEAD_PWM;
	if(fabs(angle)>30)
	{
		pwmL=0;
		pwmR=0;
	}
	
	pwmOutputR(pwmR);
	pwmOutputL(pwmL);

	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	count++;
}

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    delay_init();
    uart_init();

    
    MPU_Init();
    delay_ms(5);
    printf("MPU6050 is going to wake up\r\n");
    MPU_WakeUp();
    MPU_isReady()?printf("MPU6050 is working\r\n"):printf("MPU6050 failed to start!\r\n");
    delay_ms(5);
    MPU_SetScale(GYRO_SCALE_500,ACC_SCALE_2G);
    printf("Scale setting completed!\r\n");
    tim_pwm_init();
    tim_encoder_init();
     tim_irq_init();   
//    pwmOutputR(200);
//		pwmOutputL(600);
    while(1)
    {       

    }
    
}
