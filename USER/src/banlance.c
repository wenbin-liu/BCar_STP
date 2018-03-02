#include "math.h"
#include "stdint.h"
#include "mpu6050.h"
#include "stm32f10x_tim.h"

#define GYRO_SCALE 500
#define GYRO_BIASE (-67)
#define TOTAL_MAX 500
float SPEED_KP=3;
float SPEED_KI=0.6;
float SPEED_KD=0.0;
float BALANCE_KP=110;
float BALANCE_KD=0.6;
float DIRECTION_KP=1.6;
float DIRECTION_KD=0.03;

void getAngle(float *angle,float *gyro)
{
    int16_t ax,az,gy;
    MPU_ReadAx(&ax);
    MPU_ReadAz(&az);
    MPU_ReadGy(&gy);
    *gyro=-(gy-GYRO_BIASE)*GYRO_SCALE/32767.0;
    *angle=57.296*atan((float)ax/az);
    return;
}

void pwmOutputR(int pwm)
{
    if(pwm>0)
    {
        if(pwm>720)
            pwm=720;
				TIM_SetCompare1(TIM2,0);
				TIM_SetCompare2(TIM2,pwm);
				

    }

    else
    {
        if(pwm<-720)
            pwm=-720;
				TIM_SetCompare2(TIM2,0);
				TIM_SetCompare1(TIM2,-pwm);
    }
}
void pwmOutputL(int pwm)
{
    if(pwm>0)
    {
        if(pwm>720)
            pwm=720;
				TIM_SetCompare3(TIM2,0);
				TIM_SetCompare4(TIM2,pwm);
    }

    else
    {
        if(pwm<-720)
            pwm=-720;
				TIM_SetCompare4(TIM2,0);
				TIM_SetCompare3(TIM2,-pwm);
    }
}
int balanceControl(float angle_d,float angle ,float gyro)
{
    return (int)BALANCE_KP*(angle_d-angle)+BALANCE_KD*(-gyro);
}

int speedControl(int spd_d,int spdL,int spdR,int running)
{
    static float total,err;
		float last=err;
    err=(spd_d-(spdL+spdR)/2);
    total+=err;
    if(total>TOTAL_MAX)
        total=TOTAL_MAX;
    else if(total<-TOTAL_MAX)
        total=-TOTAL_MAX;
		if(running!=0)
				total=0;
		
    return err*SPEED_KP+total*SPEED_KI+(err-last)*SPEED_KD;
 
}

int directionControl(int w_d,int w)
{
	static int err;
	int last=err;
	err=w_d-w;
	return DIRECTION_KP*err+DIRECTION_KD*(err-last);
}
//int smoothPwmSpd(int spd)
//{
//	static int count;
//	static float pwm;
//	count++;
//	pwm
//}
