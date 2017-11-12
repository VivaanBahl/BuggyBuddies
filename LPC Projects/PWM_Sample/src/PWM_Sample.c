/*
===============================================================================
 Name        : PWM_Sample.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "board.h"
#include <stdio.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
///**
// * @brief	Handle interrupt from SysTick timer
// * @return	Nothing
// */
void SysTick_Handler(void)
{
	Board_LED_Set(0, false);
}

/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing
 */
void TIMER16_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER16_0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER16_0, 1);
		Board_LED_Toggle(0);
	}
}

/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{

	SystemCoreClockUpdate();
	Board_Init();

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock);

	/* Enable timer 1 clock */
	Chip_TIMER_Init(LPC_TIMER16_0);

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER16_0);
	LPC_TIMER16_0->MR[0] = 0;
	LPC_TIMER16_0->MR[3] = 5000;
	LPC_TIMER16_0->PR = 48;
	LPC_TIMER16_0->MCR = 0;
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_0, 3);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC2 | IOCON_MODE_INACT));
	LPC_TIMER16_0->PWMC = 0x1;
	Chip_TIMER_Enable(LPC_TIMER16_0);


	/* LEDs toggle in interrupt handlers */

	while (1) {
		for (int i = 0; i < 1500; i++)
		{
			steering_set(i);
		}
		for (int i = 0; i > -1500; i--)
		{
			steering_set(i);
		}
	}

	return 0;
}
