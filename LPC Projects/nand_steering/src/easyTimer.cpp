/*
 * easyTimer.cpp
 *
 *  Created on: Mar 26, 2018
 * 	  Modified:	Apr 2, 2018
 *      Author: Adam Quinn
 */

#include <stdint.h>
#include "easyTimer.h"
#include "chip.h"
#include "board.h"


extern "C" {

	void SysTick_Handler(void)
	{
		ET.increment();
	}

}

void easyTimer::InitEasyTimer()
{
	if(this->initialized==false)
	{
		SystemCoreClockUpdate();
		Board_Init();
		//Set up the SysTick timer to trigger every millisecond.
		SysTick_Config(SystemCoreClock/1000);
		this->currentMillis = 0;
		this->initialized = true;
	}

}

void easyTimer::delay(uint32_t milliseconds)
{
	if(!this->initialized) this->InitEasyTimer();
	uint32_t oldTime = millis() + milliseconds;

	while(millis() < oldTime) {}
}

uint32_t easyTimer::millis()
{
	return this->currentMillis;
}

void easyTimer::increment()
{
	this->currentMillis++;
}

easyTimer ET;


