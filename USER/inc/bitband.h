/*
 * bitband.h
 *
 *  Created on: 2017��9��14��
 *      Author: Liu
 */

#ifndef INC_BITBAND_H_
#define INC_BITBAND_H_
#include "stm32f1xx.h"

#define BITBAND(ADDR,BIT) (((ADDR-((ADDR)&0xF0000000UL))<<5)+((BIT)<<2)+(ADDR&0xF0000000UL)+0x02000000UL)
#define PCOUT(N) (*((uint32_t *)BITBAND(GPIOC_BASE+0x0CUL,N)))
#define PAOUT(N) (*((uint32_t *)BITBAND(GPIOA_BASE+0x0CUL,N)))
#define PBOUT(N) (*((uint32_t *)BITBAND(GPIOB_BASE+0x0CUL,N)))
#endif /* INC_BITBAND_H_ */
