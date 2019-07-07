#include "stm32f10x.h"
#include "stm32-ints.h"

#define bit_set(var,bitno) ((var) |= 1 << (bitno))
#define bit_clr(var,bitno) ((var) &= ~(1 << (bitno)))
#define testbit(var,bitno) (((var)>>(bitno)) & 0x01)

#define BUF_SIZE 14
char data_to_send[BUF_SIZE] = "DMA transfer\n\r";

void delay(void) 
{
    int i = 100000;
    while (i-- > 0) {
        asm("nop");
    }
}

void uart_init(void)
{
    /* PA9 - UART1_TX -> output 10MHz, Alternate function output push-pull */
    bit_set(GPIOA->CRH, 7);
    bit_clr(GPIOA->CRH, 6);
    bit_clr(GPIOA->CRH, 5);
    bit_set(GPIOA->CRH, 4);

    bit_set(RCC->APB2ENR, 14); /* enable USART1 clock */
    USART1->BRR = 0x85; /* 8 mantissa, 5 fraction -> 115200 baud */
    bit_set(USART1->CR1, 15); /* enable OVER8 = 1  */
    bit_set(USART1->CR1, 13); /* enable USART1 */
    bit_set(USART1->CR1, 3); /* enable USART1 TX */
}

void uart1_dma_set(char *src, int len, int circular_mode)
{
    bit_set(RCC->AHBENR, 0); /* enable DMA1 clock */
    bit_set(USART1->CR3, 7); /* enable USART1 TX DMA */
    bit_clr(USART1->SR, 6); /* clear UART transmission complete flag */

    /* USART1_TX is connected to DMA1 channel 4, configuring channel */
    DMA_Channel4->CCR = 0; /* clear control register */
    bit_set(DMA_Channel4->CCR, 7); /* memory increment mode */
    bit_set(DMA_Channel4->CCR, 4); /* memory -> peripheral */
    if(circular_mode) bit_set(DMA_Channel4->CCR, 5); /* enable circular mode */
    DMA_Channel4->CNDTR = len; /* nr of transfer data */
    DMA_Channel4->CPAR = (unsigned int) &(USART1->DR); /* peripheral addr */
    DMA_Channel4->CMAR = (unsigned int) src; /* memory addr */
    bit_set(DMA_Channel4->CCR, 0); /* channel enable */
}

void main(void){
    int button_press = 0, button_stable = 0;

    RCC->APB2ENR |= 0x10 | 0x04; /* Enable the GPIOA (bit 2) and GPIOC (bit 8) */
    GPIOC->CRH = 0x11; /* Set GPIOC Pin 8 and Pin 9 to outputs */
    GPIOA->CRL = 0x04; /* Set GPIOA Pin 0 to input floating */

    uart_init();
    uart1_dma_set(data_to_send, BUF_SIZE, 0);

    while(1)
    {
	while(!button_press) /* wait for user button to be pressed */
	    button_press = ((GPIOA->IDR & 0x1) == 1);

	while(button_stable < 2) /* debounce button state and wait for button release */
	{
	    button_press = ((GPIOA->IDR & 0x1) == 1);
	    if(!button_press) button_stable++;
	    else button_stable = 0;
	    delay();
	}
	button_stable = 0;

	if(DMA_Channel4->CNDTR == 0)
	{
	    data_to_send[0]++;
	    if(data_to_send[0] == ('Z' + 1))
		data_to_send[0] = 'A';

	    bit_clr(DMA_Channel4->CCR, 0); /* channel disable */
	    DMA_Channel4->CNDTR = BUF_SIZE;
	    bit_set(DMA_Channel4->CCR, 0); /* channel enable */
	}
    }
}
