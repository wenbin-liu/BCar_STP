#ifndef __DELAY_H
#define __DELAY_H 		
#include <stdint.h>
#include "stm32f10x.h"
void delay_init();
void delay_ms(uint16_t nms);
void delay_us(uint16_t nus);

#endif





























