#include "delay.h"

void delay_init()
{
    SysTick->LOAD=72000;
    SysTick->CTRL=SysTick_CTRL_CLKSOURCE|SysTick_CTRL_ENABLE;
}

void delay_ms(uint16_t ms)
{
    SysTick->VAL=0;
    while(ms>0)
        if((SysTick->CTRL&SysTick_CTRL_COUNTFLAG)!=0)
            ms--;
}

void delay_us(uint16_t us)
{
    int t=SysTick->LOAD;
    SysTick->LOAD=us*72;
    SysTick->VAL=0;
    while((SysTick->CTRL&SysTick_CTRL_COUNTFLAG) ==0)
        ;
    SysTick->LOAD=t; 
}
