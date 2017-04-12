/*
 * main.cpp
 *
 *  Created on: 09 марта 2017 г.
 *      Author: Alekseu
 */

#include "main.h"

#include "La/sump.h"

#include "StdPeriph/stm_lib/inc/misc.h"
#include "StdPeriph/stm_lib/inc/stm32f10x_tim.h"
#include "StdPeriph/stm_lib/inc/stm32f10x_rcc.h"

#include "Driver/usb/usb.h"

void Overclocking(void) // Разгон микроконтроллера.
{

  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE); // Выбираем источником такторования
  RCC_PLLCmd(DISABLE); // Выключаем умножитель.
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_CFGR_PLLMULL16); // На сколько будем умножать частоту.
  RCC_PLLCmd(ENABLE); // Включаем умножитель.
  while ((RCC->CR & RCC_CR_PLLRDY) == 0);     // Ждем запуска умножителя.
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // Выбираем источником тактирования умножитель.

  SystemCoreClockUpdate(); // Вычисление тактовой частоты ядра.
}



Usb com;



//typedef void reset__(void);
//reset__* reset_ = 0;

void write_byte(uint8_t data)
{
	com.WriteByte(data);
}

void write_buffer(uint8_t *data, int count)
{
	com.SendData(data,count);
}




int main()
{

	_leds.Init();

	SumpSetTXFunctions(write_byte,write_buffer);

	//Overclocking();
	com.RxBufferSize = 64;
	com.TxBufferSize = 64;
	com.TypeUsb = VirtualComPort;
	com.OnRecived = SumpProcessRequest;
	com.Init();




	while(1){

		_leds.On(1);
		_delay_ms(150);
		_leds.Off(1);
		_delay_ms(150);
	};

	return 0;
}
