#ifndef __LCD_H
#define __LCD_H

#include "stm32f1xx_hal.h"


#define LCD_RST GPIO_PIN_4
#define LCD_CE GPIO_PIN_3
#define LCD_DC GPIO_PIN_2
#define LCD_DIN GPIO_PIN_1
#define LCD_CLK GPIO_PIN_0
#define LCD_BL GPIO_PIN_5
#define LCD_XADDR_MAX 83
#define LCD_YADDR_MAX 5
#define LCD_XADDR_LEN (LCD_XADDR_MAX+1)
#define LCD_YADDR_LEN (LCD_YADDR_MAX+1)
#define LCD_ADDR_MAX 503
#define LCD_PDVH_MSK 0x20U
#define WAIT() delay_us(1)
#define HIGH 1
#define LOW 0

void delay_us(uint32_t);
void writePin(uint32_t pin,uint8_t state);
void LCD_Init(void);
void LCD_Draw(uint8_t x,uint8_t y,uint8_t data);
void LCD_Start(void);
void LCD_Write(uint8_t,uint8_t);
void LCD_Clear(void);
void LCD_Putchar(uint8_t c);
void LCD_SetCursor(uint8_t x,uint8_t y);

#endif
