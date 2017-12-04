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

#define TEST_CCAN_BAUD_RATE 1000000
#define PWM_SWITCH_THRESHOLD 1750
CCAN_MSG_OBJ_T msg_obj;

uint16_t k_min_pulse = 500;
uint16_t k_max_pulse = 2500;
uint16_t k_offset_rc_in = 1500;
uint16_t k_scale_rc_in = 350;
uint16_t k_offset_stored_angle = 0;
uint16_t k_scale_stored_angle = 1500;

uint16_t SteeringRiseTime = 0;
uint16_t PulseWidth_US = 0;

uint16_t BrakeRiseTime = 0;
bool brake = true;

uint32_t AutonRiseTime = 0;
bool auton = false;

void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

void CAN_tx(uint8_t msg_obj_num) {
	Board_UARTPutSTR("Sent\n");
}

void CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);

	int received = 0;
	received |= (uint8_t)msg_obj.data[0];
	received |= ((uint8_t)msg_obj.data[1] << 8);
	received |= ((uint8_t)msg_obj.data[2] << 16);
	received |= ((uint8_t)msg_obj.data[3] << 24);
}

void CAN_error(uint32_t error_info) {
	Board_UARTPutSTR("CAN error!\n");
}

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

long map(long x,
		long in_offset,
		long in_scale,
		long out_offset,
		long out_scale) {
	return ((x - in_offset) * out_scale / in_scale) + out_offset;
}

// Steering PWD
void TIMER16_0_IRQHandler(void)
{
	bool CurrentValue = Chip_GPIO_GetPinState(LPC_GPIO, 0, 2);
	uint16_t CurrentTimer = LPC_TIMER16_0->CR[0];
	if (CurrentValue)
	{
		SteeringRiseTime = CurrentTimer;
	}
	else
	{
		if (SteeringRiseTime != 0)
		{
			if (CurrentTimer > k_min_pulse && CurrentTimer < k_max_pulse)
			{
				/* Send a simple one time CAN message */
				PulseWidth_US = CurrentTimer;
				int value = (int)map(PulseWidth_US,
						k_offset_rc_in,
						k_scale_rc_in,
						k_offset_stored_angle,
						k_scale_stored_angle);
				msg_obj.msgobj  = 0;
				msg_obj.mode_id = 0x100;
				msg_obj.mask    = 0x0;
				msg_obj.dlc     = 4;
				msg_obj.data[0] = value & 0xFF;
				msg_obj.data[1] = (value >> 8) & 0xFF;
				msg_obj.data[2] = (value >> 16) & 0xFF;
				msg_obj.data[3] = (value >> 24) & 0xFF;
				LPC_CCAN_API->can_transmit(&msg_obj);

			}
		}
	}
	LPC_TIMER16_0->TC = 0;
	if (Chip_TIMER_CapturePending(LPC_TIMER16_0, 0))
	{
		Chip_TIMER_ClearCapture(LPC_TIMER16_0, 0);
	}
}

// Brake PWD
void TIMER16_1_IRQHandler(void)
{
	bool CurrentValue = Chip_GPIO_GetPinState(LPC_GPIO, 1, 8);
	uint16_t CurrentTimer = LPC_TIMER16_1->CR[0];
	if (CurrentValue)
	{
		BrakeRiseTime = CurrentTimer;
	}
	else
	{
		if (BrakeRiseTime != 0)
		{
			brake = CurrentTimer > PWM_SWITCH_THRESHOLD;
		}
	}
	LPC_TIMER16_1->TC = 0;
	if (Chip_TIMER_CapturePending(LPC_TIMER16_1, 0))
	{
		Chip_TIMER_ClearCapture(LPC_TIMER16_1, 0);
	}
}

// Auton PWD
void TIMER32_0_IRQHandler(void)
{
	bool CurrentValue = Chip_GPIO_GetPinState(LPC_GPIO, 1, 5);
	uint16_t CurrentTimer = LPC_TIMER32_0->CR[0];
	if (CurrentValue)
	{
		AutonRiseTime = CurrentTimer;
	}
	else
	{
		if (AutonRiseTime != 0)
		{
			auton = CurrentTimer > PWM_SWITCH_THRESHOLD;
		}
	}
	LPC_TIMER32_0->TC = 0;
	if (Chip_TIMER_CapturePending(LPC_TIMER32_0, 0))
	{
		Chip_TIMER_ClearCapture(LPC_TIMER32_0, 0);
	}
}

int main(void)
{
	uint32_t CanApiClkInitTable[2];
	/* Publish CAN Callback Functions */
	CCAN_CALLBACKS_T callbacks = {
			CAN_rx,
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
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);

	Chip_GPIO_Init(LPC_GPIO);
	// Input pins for the receiver
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 2);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 1, 8);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 1, 5);

	// Uart setup
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	Board_UARTPutSTR("Hello World! This is Radio Receiver!\n");

	// CAN setup
	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

	msg_obj.msgobj = 0;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x700;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);


	// Set PIO0_2 to capture edges for the PWM signal
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC2 | IOCON_MODE_PULLDOWN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_8, (IOCON_FUNC1 | IOCON_MODE_PULLDOWN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_5, (IOCON_FUNC2 | IOCON_MODE_PULLDOWN));

	// Timer for capture control register for demodulating steering PWM signal from remote control
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

	// Timer for determining if brake up or down
	Chip_TIMER_Init(LPC_TIMER16_1);
	Chip_TIMER_Reset(LPC_TIMER16_1);
	// Capture timer value on rising and falling edge, and generate interrupt
	Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER16_1, 0);
	Chip_TIMER_CaptureFallingEdgeEnable(LPC_TIMER16_1, 0);
	Chip_TIMER_CaptureEnableInt(LPC_TIMER16_1, 0);
	LPC_TIMER16_1->PR = 48;
	Chip_TIMER_Enable(LPC_TIMER16_1);
	NVIC_ClearPendingIRQ(TIMER_16_1_IRQn);
	NVIC_EnableIRQ(TIMER_16_1_IRQn);

	// Timer for determining if in auton mode
	Chip_TIMER_Init(LPC_TIMER32_0);
	Chip_TIMER_Reset(LPC_TIMER32_0);
	// Capture timer value on rising and falling edge, and generate interrupt
	Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER32_0, 0);
	Chip_TIMER_CaptureFallingEdgeEnable(LPC_TIMER32_0, 0);
	Chip_TIMER_CaptureEnableInt(LPC_TIMER32_0, 0);
	LPC_TIMER32_0->PR = 48;
	Chip_TIMER_Enable(LPC_TIMER32_0);
	NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);
	NVIC_EnableIRQ(TIMER_32_0_IRQn);

	volatile static int i = 0 ;
	// Enter an infinite loop, just incrementing a counter
	while(1) {
		i++ ;
		__WFI();
	}
	return 0 ;
}
