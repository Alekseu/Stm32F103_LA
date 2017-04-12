/*
 * led.cpp
 *
 *  Created on: 31 марта 2017 г.
 *      Author: hudienko_a
 */

#include "led.h"



	Led* Led::Lobj=0;


	Led::Led()
	{
		Lobj = this;
	}

	Led::~Led()
	{

	}


	void Led::Init()
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin =  LedPin1| LedPin2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LedPort, &GPIO_InitStructure);
	}

	void Led::On(unsigned char i)
	{
		switch(i)
		{
		case 1:
			LedPort->BSRR = LedPin1;
			break;
		case 2:
			LedPort->BSRR = LedPin2;
			break;

		}
	}

	void Led::Off(unsigned char i)
	{
		switch(i)
		{
		case 1:
			LedPort->BRR = LedPin1;
			break;
		case 2:
			LedPort->BRR = LedPin2;
			break;

		}
	}

