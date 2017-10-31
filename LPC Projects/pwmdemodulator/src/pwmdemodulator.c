/*
===============================================================================
 Name        : pwmdemodulator.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

uint16_t k_min_pulse = 500;
uint16_t k_max_pulse = 2500;

uint16_t RiseTime = 0;
uint16_t PulseWidth_US = 0;

void TIMER16_0_IRQHandler(void)
{
	bool CurrentValue = Chip_GPIO_GetPinState(LPC_GPIO, 0, 2);
	uint16_t CurrentTimer = LPC_TIMER16_0->CR[0];
	if (CurrentValue)
	{
		RiseTime = CurrentTimer;
	}
	else
	{
		if (RiseTime != 0)
		{
//			char buf[100];
//			sprintf(buf, "%d\n", CurrentTimer);
//			Board_UARTPutSTR(buf);
			if (CurrentTimer > k_min_pulse && CurrentTimer < k_max_pulse)
			{
				PulseWidth_US = CurrentTimer;
				char buf[100];
				sprintf(buf, "%d\n", PulseWidth_US);
				Board_UARTPutSTR(buf);
			}
		}
	}
	LPC_TIMER16_0->TC = 0;
	if (Chip_TIMER_CapturePending(LPC_TIMER16_0, 0))
	{
		Chip_TIMER_ClearCapture(LPC_TIMER16_0, 0);
	}
}

int main(void)
{

    SystemCoreClockUpdate();
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);

    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 2);

    // Uart crap
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	Board_UARTPutSTR("asdfadsfadfasdf\n");

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC2 | IOCON_MODE_PULLDOWN));

    // Timer for capture control register
	Chip_TIMER_Init(LPC_TIMER16_0);
	Chip_TIMER_Reset(LPC_TIMER16_0);
	// Capture timer value on rising and falling edge, and generate interrupt
	Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER16_0, 0);
	Chip_TIMER_CaptureFallingEdgeEnable(LPC_TIMER16_0, 0);
	Chip_TIMER_CaptureEnableInt(LPC_TIMER16_0, 0);
	LPC_TIMER16_0->PR = 48;
	Chip_TIMER_Enable(LPC_TIMER16_0);
	NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
	NVIC_EnableIRQ(TIMER_16_0_IRQn);

    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
        __WFI();
    }
    return 0 ;
}
