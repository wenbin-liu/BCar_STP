
//#include "stm32f1xx_hal.h"
//#include "lcd.h"
//#include "Font_code.h"
//void delay_us(uint32_t t)
//{
//	t=SysTick->LOAD-72*t;
//	SysTick->VAL=0;
//	while(SysTick->VAL>t);

//}
//void writePin(uint32_t pin,uint8_t state)
//{
//	HAL_GPIO_WritePin(GPIOA,pin,state);
//}
//void LCD_Reset()
//{
//	writePin(LCD_CE,GPIO_PIN_RESET);
//	WAIT();
//	writePin(LCD_RST,GPIO_PIN_RESET);
//	WAIT();
//	writePin(LCD_RST,GPIO_PIN_SET);
//	WAIT();
//	writePin(LCD_CE,GPIO_PIN_SET);
//	WAIT();
//}


//void LCD_Init()
//{
//	//GPIO Init
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	GPIO_InitTypeDef GPIOA_InitStr=
//	{
//			.Pin=LCD_RST|LCD_CE|LCD_CLK|LCD_DC|LCD_DIN|LCD_BL,
//			.Mode=GPIO_MODE_OUTPUT_PP,
//			.Speed=GPIO_SPEED_FREQ_HIGH
//	};
//	HAL_GPIO_Init(GPIOA,&GPIOA_InitStr);
//	writePin(LCD_RST|LCD_CE|LCD_CLK|LCD_DC|LCD_DIN|LCD_BL,GPIO_PIN_SET);
//	LCD_Reset();


//}
//void LCD_Write(uint8_t data,uint8_t d_c)
//{
//	writePin(LCD_CE|LCD_CLK,LOW);
//	writePin(LCD_DC,d_c);
//	uint8_t t=8;
//	WAIT();
//	while(t--)
//	{
//		writePin(LCD_CLK,LOW);
//		WAIT();
//		writePin(LCD_DIN,(data&0x80)?1:0);
//		data<<=1;
//		WAIT();
//		writePin(LCD_CLK,HIGH);
//	}
//	writePin(LCD_CE,HIGH);


//}
//void LCD_Clear()
//{
//	for(int i=0;i<504;i++)
//	LCD_Write(0x00U,1);
//	LCD_SetCursor(0, 0);
//}
//void LCD_Start()
//{
//	writePin(LCD_CE,LOW);
//	LCD_Write(0x21U,0);
//	LCD_Write(0xC8U,0);
////	LCD_Write(0x21,0);		//����ģʽ, ˮƽѰַ, ��չָ��
//	LCD_Write(0x06,0);		//VLCD�¶�ϵ��2
//	LCD_Write(0x13,0);		//����ƫ��ϵͳ(BSx) 1:48
////	LCD_Write(0xc8,0);


//	LCD_Write(0x20U,0);
//	LCD_Write(0x0CU,0);		//����ģʽ, ˮƽѰַ, ��չָ��
//	writePin(LCD_CE,HIGH);
//	LCD_Clear();
//}

//void LCD_SetCursor(uint8_t x_addr,uint8_t y_addr)
//{
//	LCD_Write(0x80U|x_addr,0);
//	LCD_Write(0x40U|y_addr,0);
//}

//void LCD_Draw(uint8_t x,uint8_t y,uint8_t data)
//{
//	LCD_SetCursor(x,y);
//	LCD_Write(data, 1);
//}
//void LCD_Putchar(uint8_t c)
//{
//	static uint16_t addr=0;
//	if(c=='\n')
//	{
//		addr+=LCD_XADDR_LEN;
//		addr=addr/LCD_XADDR_LEN*LCD_XADDR_LEN;
//		if(addr>LCD_ADDR_MAX)
//		{
//			addr=0;
//			LCD_Clear();
//		}
//		LCD_SetCursor(0,addr/LCD_XADDR_LEN);

//	}
//	else

//		{
//			if(addr>LCD_ADDR_MAX)
//			{
//				addr-=LCD_ADDR_MAX+1;
//				LCD_Clear();
//			}
//			for(uint8_t i=0;i<6;i++)
//				LCD_Write(Font_code[c-32][i], 1);
//			addr+=6;


//		}
//}
