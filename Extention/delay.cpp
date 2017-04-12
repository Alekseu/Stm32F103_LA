/*
 * delay.cpp
 *
 *  Created on: 29 ���� 2016 �.
 *      Author: hudienko_a
 */

#include"delay.h"
#include "../StdPeriph/stm_lib/inc/stm32f10x_rcc.h"

void _delay_ms(int ms)
{
	   volatile uint32_t nCount;
	   RCC_ClocksTypeDef RCC_Clocks;
	   	RCC_GetClocksFreq(&RCC_Clocks);
		        nCount=(RCC_Clocks.HCLK_Frequency/20000)*ms;
		        for (; nCount!=0; nCount--);
}


