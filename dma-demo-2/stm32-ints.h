#define STACK_TOP 0x20002000										/* This can move quite a lot ! */

/* Vector Table */

void Reset_Handler (void) __attribute__ ((weak));
void NMI_Handler (void) __attribute__ ((weak));
void HardFault_Handler (void) __attribute__ ((weak));
void MemManage_Handler (void) __attribute__ ((weak));
void BusFault_Handler (void) __attribute__ ((weak));
void UsageFault_Handler (void) __attribute__ ((weak));
void SVC_Handler (void) __attribute__ ((weak));
void DebugMon_Handler (void) __attribute__ ((weak));
void PendSV_Handler (void) __attribute__ ((weak));
void SysTick_Handler (void) __attribute__ ((weak));
void WWDG_IRQHandler (void) __attribute__ ((weak));
void PVD_IRQHandler (void) __attribute__ ((weak));
void TAMPER_STAMP_IRQHandler (void) __attribute__ ((weak));
void RTC_WKUP_IRQHandler (void) __attribute__ ((weak));
void FLASH_IRQHandler (void) __attribute__ ((weak));
void RCC_IRQHandler (void) __attribute__ ((weak));
void EXTI0_IRQHandler (void) __attribute__ ((weak));
void EXTI1_IRQHandler (void) __attribute__ ((weak));
void EXTI2_IRQHandler (void) __attribute__ ((weak));
void EXTI3_IRQHandler (void) __attribute__ ((weak));
void EXTI4_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel1_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel2_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel3_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel4_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel5_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel6_IRQHandler (void) __attribute__ ((weak));
void DMA1_Channel7_IRQHandler (void) __attribute__ ((weak));
void ADC1_IRQHandler (void) __attribute__ ((weak));
void EXTI9_5_IRQHandler (void) __attribute__ ((weak));
void TIM1_BRK_TIM15_IRQHandler (void) __attribute__ ((weak));
void TIM1_UP_TIM16_IRQHandler (void) __attribute__ ((weak));
void TIM1_TRG_COM_TIM17_IRQHandler (void) __attribute__ ((weak));
void TIM1_CC_IRQHandler (void) __attribute__ ((weak));
void TIM2_IRQHandler (void) __attribute__ ((weak));
void TIM3_IRQHandler (void) __attribute__ ((weak));
void TIM4_IRQHandler (void) __attribute__ ((weak));
void I2C1_EV_IRQHandler (void) __attribute__ ((weak));
void I2C1_ER_IRQHandler (void) __attribute__ ((weak));
void I2C2_EV_IRQHandler (void) __attribute__ ((weak));
void I2C2_ER_IRQHandler (void) __attribute__ ((weak));
void SPI1_IRQHandler (void) __attribute__ ((weak));
void SPI2_IRQHandler (void) __attribute__ ((weak));
void USART1_IRQHandler (void) __attribute__ ((weak));
void USART2_IRQHandler (void) __attribute__ ((weak));
void USART3_IRQHandler (void) __attribute__ ((weak));
void EXTI15_10_IRQHandler (void) __attribute__ ((weak));
void RTCAlarm_IRQHandler (void) __attribute__ ((weak));
void CEC_IRQHandler (void) __attribute__ ((weak));
void TIM12_IRQHandler (void) __attribute__ ((weak));
void TIM13_IRQHandler (void) __attribute__ ((weak));
void TIM14_IRQHandler (void) __attribute__ ((weak));
void ADC3_IRQHandler (void) __attribute__ ((weak));
void FSMC_IRQHandler (void) __attribute__ ((weak));
void TIM5_IRQHandler (void) __attribute__ ((weak));
void SPI3_IRQHandler (void) __attribute__ ((weak));
void UART4_IRQHandler (void) __attribute__ ((weak));
void UART5_IRQHandler (void) __attribute__ ((weak));
void TIM6_DAC_IRQHandler (void) __attribute__ ((weak));
void TIM7_IRQHandler (void) __attribute__ ((weak));
void DMA2_Channel1_IRQHandler (void) __attribute__ ((weak));
void DMA2_Channel2_IRQHandler (void) __attribute__ ((weak));
void DMA2_Channel3_IRQHandler (void) __attribute__ ((weak));
void DMA2_Channel4_5_IRQHandler (void) __attribute__ ((weak));
void DMA2_Channel5_IRQHandler (void) __attribute__ ((weak));

unsigned int * int_vectors[] 
__attribute__ ((section(".vectors")))= {
    (unsigned int *) STACK_TOP,         
	(unsigned int *) Reset_Handler,
	(unsigned int *) NMI_Handler,
	(unsigned int *) HardFault_Handler,
	(unsigned int *) MemManage_Handler,
	(unsigned int *) BusFault_Handler,
	(unsigned int *) UsageFault_Handler,
	0, 0, 0, 0,
	(unsigned int *) SVC_Handler,
	(unsigned int *) DebugMon_Handler,
	0,
	(unsigned int *) PendSV_Handler,
	(unsigned int *) SysTick_Handler,
	(unsigned int *) WWDG_IRQHandler,
	(unsigned int *) PVD_IRQHandler,
	(unsigned int *) TAMPER_STAMP_IRQHandler,
	(unsigned int *) RTC_WKUP_IRQHandler,
	(unsigned int *) FLASH_IRQHandler,
	(unsigned int *) RCC_IRQHandler,
	(unsigned int *) EXTI0_IRQHandler,
	(unsigned int *) EXTI1_IRQHandler,
	(unsigned int *) EXTI2_IRQHandler,
	(unsigned int *) EXTI3_IRQHandler,
	(unsigned int *) EXTI4_IRQHandler,
	(unsigned int *) DMA1_Channel1_IRQHandler,
	(unsigned int *) DMA1_Channel2_IRQHandler,
	(unsigned int *) DMA1_Channel3_IRQHandler,
	(unsigned int *) DMA1_Channel4_IRQHandler,
	(unsigned int *) DMA1_Channel5_IRQHandler,
	(unsigned int *) DMA1_Channel6_IRQHandler,
	(unsigned int *) DMA1_Channel7_IRQHandler,
	(unsigned int *) ADC1_IRQHandler,
	0, 0, 0, 0,
	(unsigned int *) EXTI9_5_IRQHandler,
	(unsigned int *) TIM1_BRK_TIM15_IRQHandler,
	(unsigned int *) TIM1_UP_TIM16_IRQHandler,
	(unsigned int *) TIM1_TRG_COM_TIM17_IRQHandler,
	(unsigned int *) TIM1_CC_IRQHandler,
	(unsigned int *) TIM2_IRQHandler,
	(unsigned int *) TIM3_IRQHandler,
	(unsigned int *) TIM4_IRQHandler,
	(unsigned int *) I2C1_EV_IRQHandler,
	(unsigned int *) I2C1_ER_IRQHandler,
	(unsigned int *) I2C2_EV_IRQHandler,
	(unsigned int *) I2C2_ER_IRQHandler,
	(unsigned int *) SPI1_IRQHandler,
	(unsigned int *) SPI2_IRQHandler,
	(unsigned int *) USART1_IRQHandler,
	(unsigned int *) USART2_IRQHandler,
	(unsigned int *) USART3_IRQHandler,
	(unsigned int *) EXTI15_10_IRQHandler,
	(unsigned int *) RTCAlarm_IRQHandler,
    (unsigned int *) CEC_IRQHandler,
	(unsigned int *) TIM12_IRQHandler,
	(unsigned int *) TIM13_IRQHandler,
	(unsigned int *) TIM14_IRQHandler,
	0,
	(unsigned int *) ADC3_IRQHandler,
	(unsigned int *) FSMC_IRQHandler,
	0,
	(unsigned int *) TIM5_IRQHandler,
	(unsigned int *) SPI3_IRQHandler,
	(unsigned int *) UART4_IRQHandler,
	(unsigned int *) UART5_IRQHandler,
	(unsigned int *) TIM6_DAC_IRQHandler,
	(unsigned int *) TIM7_IRQHandler,
	(unsigned int *) DMA2_Channel1_IRQHandler,
	(unsigned int *) DMA2_Channel2_IRQHandler,
	(unsigned int *) DMA2_Channel3_IRQHandler,
	(unsigned int *) DMA2_Channel4_5_IRQHandler,
	(unsigned int *) DMA2_Channel5_IRQHandler,
	0, 0, 0, 0, 0, 0, 0, 0,
	0,
};
