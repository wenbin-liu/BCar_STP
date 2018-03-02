#ifndef __BANLANCE_H
#define __BANLANCE_H

void getAngle(float *angle,float *gyro);
void pwmOutputR(int pwm);
void pwmOutputL(int pwm);
int balanceControl(float angle_d,float angle ,float gyro);
int speedControl(int spd_d,int spdL,int spdR,int running);
int directionControl(int w_d,int w);

#endif