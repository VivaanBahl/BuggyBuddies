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

/* GPIO pins for encoder interrupt */
#define PIN_A					2 /* GPIO pin number mapped to PIN_A */
#define PIN_A_PORT				0 /* GPIO port number mapped to PIN_A */
#define PIN_B					5 /* GPIO pin number mapped to PIN_B */
#define PIN_B_PORT				0 /* GPIO port number mapped to PIN_B */
#define IOCON_PIN_A_ID			IOCON_PIO0_2 /* IOCON pin identifer */
#define IOCON_PIN_B_ID			IOCON_PIO0_3 /* IOCON pin identifer */
#define ENCODER_IRQ_HANDLER		PIOINT0_IRQHandler /* GPIO interrupt IRQ function name */
#define ENCODER_NVIC_NAME		EINT0_IRQn /* GPIO interrupt NVIC interrupt name */

#include <cr_section_macros.h>

volatile uint8_t last_state;
volatile long ticks;

volatile int desired_steering_angle = 0;
volatile int direction = 1; // +1 forwards, -1 backwards


void ENCODER_IRQ_HANDLER(void) {

	Chip_GPIO_ClearInts(LPC_GPIO, PIN_A_PORT, (1 << PIN_A));
	Chip_GPIO_ClearInts(LPC_GPIO, PIN_B_PORT, (1 << PIN_B));

	// Encoder logic here
	bool pin_a_new = Chip_GPIO_GetPinState(LPC_GPIO, PIN_A_PORT, PIN_A);
	bool pin_b_new = Chip_GPIO_GetPinState(LPC_GPIO, PIN_B_PORT, PIN_B);
	uint8_t new_state = ((last_state << 2) | ((uint8_t)pin_a_new << 1) | ((uint8_t)pin_b_new)) & 0x0F;

	switch (new_state) {
/*
 *      old A B new A B
        0000 (0): no change
        0001 (1): reverse
        0010 (2): forward
        0011 (3): reverse x2 (assume A interrupt only)
        0100 (4): forward
        0101 (5): no change
        0110 (6): forward x2 (assume A interrupt only)
        0111 (7): reverse
        1000 (8): reverse
        1001 (9): forward x2 (assume A interrupt only)
        1010 (10): no change
        1011 (11): forward
        1100 (12): reverse x2 (assume A interrupt only)
        1101 (13): forward
        1110 (14): reverse
        1111 (15): no change
 */
	// TODO: just going to assume that we always get one bit change on interrupt for now, so need to handle weird cases

	// forward
	case 2:
	case 4:
	case 11:
	case 13:
		ticks++;
		break;
	case 3:
	case 12:
		Board_UARTPutSTR("edge backwards\n");
		ticks -= 2;
		break;
	case 9:
	case 6:
		Board_UARTPutSTR("edge forward\n");
		ticks += 2;
		break;

	// reverse
    case 1:
    case 7:
    case 8:
    case 14:
        ticks--;
        break;
	}



	last_state = new_state;
	return;
}
///**
// * @brief	Handle interrupt from SysTick timer
// * @return	Nothing
// */
void SysTick_Handler(void)
{
	desired_steering_angle += (1000 * direction);

	if (desired_steering_angle >= 1500 || desired_steering_angle <= -1500)
	{
		direction = -1*direction;
		desired_steering_angle += (1000 * direction);
	}
}

void TIMER16_1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER16_1, 0)) {
		Chip_TIMER_ClearMatch(LPC_TIMER16_1, 0);
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
	SysTick_Config(SystemCoreClock / 3);

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

	Chip_TIMER_Init(LPC_TIMER16_1);
	Chip_TIMER_Reset(LPC_TIMER16_1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_1, 0, 10000);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_1, 0);
	LPC_TIMER16_1->PR = 48;
	Chip_TIMER_Enable(LPC_TIMER16_1);
	NVIC_ClearPendingIRQ(TIMER_16_1_IRQn);
	NVIC_EnableIRQ(TIMER_16_1_IRQn);

    // Uart crap
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);


    /* Configure GPIO pins as input pins */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, PIN_A_PORT, PIN_A);
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, PIN_B_PORT, PIN_B);

	/* Configure pins as GPIO with pullup */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIN_A_ID,
		(IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIN_B_ID,
		(IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));

	/* Configure channel interrupts as edge sensitive and both edge interrupt */
	Chip_GPIO_SetupPinInt(LPC_GPIO, PIN_A_PORT, PIN_A, GPIO_INT_BOTH_EDGES);
	Chip_GPIO_SetupPinInt(LPC_GPIO, PIN_B_PORT, PIN_B, GPIO_INT_BOTH_EDGES);

	/* Enable GPIO pin interrupts */
	Chip_GPIO_EnableInt(LPC_GPIO, PIN_A_PORT, (1 << PIN_A));
	Chip_GPIO_EnableInt(LPC_GPIO, PIN_B_PORT, (1 << PIN_B));

	/* Enable wakeup for GPIO pins used in this example */
	Chip_SYSCTL_EnableStartPin(0x04); // Pin A
	Chip_SYSCTL_ResetStartPin(0x04); // Pin A
	Chip_SYSCTL_EnableStartPin(0x20); // Pin B
	Chip_SYSCTL_ResetStartPin(0x20); // Pin B

	/* Enable all clocks, even those turned off at wake power up */
	Chip_SYSCTL_SetWakeup(~(SYSCTL_SLPWAKE_IRCOUT_PD | SYSCTL_SLPWAKE_IRC_PD |
		SYSCTL_SLPWAKE_FLASH_PD | SYSCTL_SLPWAKE_SYSOSC_PD | SYSCTL_SLPWAKE_SYSOSC_PD | SYSCTL_SLPWAKE_SYSPLL_PD));

	/* Enable interrupt in the NVIC */
	NVIC_EnableIRQ(ENCODER_NVIC_NAME);

	// Read initial state
	bool pin_a = Chip_GPIO_GetPinState(LPC_GPIO, PIN_A_PORT, PIN_A);
	bool pin_b = Chip_GPIO_GetPinState(LPC_GPIO, PIN_B_PORT, PIN_B);
	last_state = (((uint8_t)pin_a << 1) | ((uint8_t)pin_b)) & 0x0F;


	/* LEDs toggle in interrupt handlers */
	while(1)
	{
		Board_LED_Toggle(0);

		steering_set(desired_steering_angle);
		__WFI();
	}

	return 0;
}
