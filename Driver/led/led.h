/*
 * Blinker.h
 *
 *  Created on: 09 марта 2017 г.
 *      Author: hudienko_a
 */

#ifndef DRIVER_BLINKER_H_
#define DRIVER_BLINKER_H_

#include "../platform.h"

	class Led
	{
	public:
		static Led* Lobj;

		Led();

		~Led();

		void Init();

		void On(unsigned char i);

		void Off(unsigned char i);

	private:
		GpioInit  GPIO_InitStructure;
	};



#endif /* DRIVER_BLINKER_H_ */
