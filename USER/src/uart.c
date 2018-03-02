#include "uart.h"

void uart_init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB,ENABLE);   
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART3EN,ENABLE);

    USART_InitTypeDef uart_Str;
    uart_Str.USART_BaudRate = 115200;
    uart_Str.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    uart_Str.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    uart_Str.USART_Parity=USART_Parity_No;
    uart_Str.USART_StopBits=USART_StopBits_1;
    uart_Str.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART3,&uart_Str);
    USART_Cmd(USART3,ENABLE);

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    NVIC_SetPriority(USART3_IRQn,11);
    NVIC_EnableIRQ(USART3_IRQn);

    GPIO_InitTypeDef gpio_str;
    gpio_str.GPIO_Mode=GPIO_Mode_AF_PP;
    gpio_str.GPIO_Pin=GPIO_Pin_10;
    gpio_str.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&gpio_str);
    gpio_str.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    gpio_str.GPIO_Pin=GPIO_Pin_11;
    GPIO_Init(GPIOB,&gpio_str);
    
}

int fputc(int c,FILE *fp)
{
    USART_SendData(USART3,(int16_t) c);    
    while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) !=SET)
            ;
    return c;
}
