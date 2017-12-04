/*
===============================================================================
 Name        : UART2CAN.c
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

#define TEST_CCAN_BAUD_RATE 1000000
CCAN_MSG_OBJ_T msg_obj;

void baudrateCalculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg)
{
	uint32_t pClk, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
	pClk = Chip_Clock_GetMainClockRate();

	clk_per_bit = pClk / baud_rate;

	for (div = 0; div <= 15; div++) {
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * (div + 1))) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					can_api_timing_cfg[0] = div;
					can_api_timing_cfg[1] =
							((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

void CAN_tx(uint8_t msg_obj_num) {
	Board_UARTPutSTR("Sent\n");
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occured on the CAN bus */
void CAN_error(uint32_t error_info) {
	Board_UARTPutSTR("Error!\n");
}

/**
 * @brief	CCAN Interrupt Handler
 * @return	Nothing
 * @note	The CCAN interrupt handler must be provided by the user application.
 *	It's function is to call the isr() API located in the ROM
 */
void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

void SysTick_Handler()
{
	Board_UARTPutSTR("hello world\n");
}
// TODO: insert other definitions and declarations here

int main(void) {

	uint32_t CanApiClkInitTable[2];
	/* Publish CAN Callback Functions */
	CCAN_CALLBACKS_T callbacks = {
			NULL,
			CAN_tx,
			CAN_error,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
	};
	SystemCoreClockUpdate();
	Board_Init();
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);
	//	SysTick_Config(SystemCoreClock / 10);

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */

	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);
	NVIC_ClearPendingIRQ(CAN_IRQn);


	while(1) {
		char c = 0;
		int steer_angle = 0;

		while (c != 0x0A)
		{
			c = Board_UARTGetChar();
			if (c == 0xFF) continue;
			if (c == '-') {
				steer_angle = -1000;
				Board_UARTPutSTR("minus\n");
			}
			else if (c == '+') {
				steer_angle = 1000;
				Board_UARTPutSTR("plus\n");
			}
			else if (c == '0') {
				steer_angle = 0;
				Board_UARTPutSTR("zero\n");
			}
		}

		msg_obj.msgobj  = 0;
		msg_obj.mode_id = 0x200;
		msg_obj.mask    = 0x0;
		msg_obj.dlc     = 4;
		msg_obj.data[0] = steer_angle & 0xFF;
		msg_obj.data[1] = (steer_angle >> 8) & 0xFF;
		msg_obj.data[2] = (steer_angle >> 16) & 0xFF;
		msg_obj.data[3] = (steer_angle >> 24) & 0xFF;
		LPC_CCAN_API->can_transmit(&msg_obj);
	}

	Board_UARTPutSTR("hello world\n");
	// TODO: insert code here

	// Force the counter to be placed into memory
	volatile static int i = 0 ;
	// Enter an infinite loop, just incrementing a counter
	while(1) {
		i++ ;
	}
	return 0 ;
}
