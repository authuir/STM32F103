#ifndef __APP__H__INCLUDED__
#define __APP__H__INCLUDED__
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "USART.h"
#include "delay.h"

extern __IO uint16_t ADCConvertedValue[6];
u8 All_Init(void);

#endif 
