#ifndef __UART_H
#define __UART_H
#include "stm32f10x_usart.h"
#include <stdio.h>

void uart_init();
int fputc(int c,FILE *fp);

#endif