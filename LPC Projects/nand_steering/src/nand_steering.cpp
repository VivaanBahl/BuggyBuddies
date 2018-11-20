/*
 ===============================================================================
 Name        : main.c
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
#include "DServo.h"
#include "easyTimer.h"
#endif
#endif

#include <cr_section_macros.h>

volatile uint8_t last_state;
volatile long ticks;

volatile int desired_steering_angle = 0;

#define SERVO_ID 0xFE               // ID of which we will set Dynamixel too
#define SERVO_RECEIVE_ID 0x04
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set to (57600)
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anti-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
#define SCALING_FACTOR 0xB			//floor of 2048/180, scaling degrees to dynamixel units

#define TEST_CCAN_BAUD_RATE 1000000
CCAN_MSG_OBJ_T msg_obj;

bool collision_detected = false;

void baudrateCalculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg) {
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
					can_api_timing_cfg[1] = ((quanta - 1) & 0x3F)
							| (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8
							| (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
 a CAN message has been received */
void CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	int new_desired_angle = /*desired_steering_angle*/5;

	if (msg_obj.mode_id == 0x400) {
		new_desired_angle = 0;
		new_desired_angle |= (msg_obj.data[0]);
		new_desired_angle |= (msg_obj.data[1]) << 8;
		new_desired_angle |= (msg_obj.data[2]) << 16;
		new_desired_angle |= (msg_obj.data[3]) << 24;
	}

	desired_steering_angle = new_desired_angle;
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

int main(void) {

	uint32_t CanApiClkInitTable[2];
	/* Publish CAN Callback Functions */

	CCAN_CALLBACKS_T callbacks = { CAN_rx,
			CAN_rx, CAN_error,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL, };
	SystemCoreClockUpdate();
	Board_Init();
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);

	/* Enable timer 1 clock */
//	Chip_TIMER_Init(LPC_TIMER16_0);

	// CAN
	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);
	NVIC_ClearPendingIRQ(CAN_IRQn);

	msg_obj.msgobj = 0;
	msg_obj.mode_id = 0x400;
	msg_obj.mask = 0x000;

	/*msg_obj.msgobj = 1;
	msg_obj.mode_id = 0 | CAN_MSGOBJ_EXT;  //receive all
	msg_obj.mask = 0x7FF;*/
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* Enable all clocks, even those turned off at wake power up */
	/*Chip_SYSCTL_SetWakeup(
			~(SYSCTL_SLPWAKE_IRCOUT_PD | SYSCTL_SLPWAKE_IRC_PD |
			SYSCTL_SLPWAKE_FLASH_PD | SYSCTL_SLPWAKE_SYSOSC_PD
					| SYSCTL_SLPWAKE_SYSOSC_PD | SYSCTL_SLPWAKE_SYSPLL_PD));*/

	//ET.InitEasyTimer();

	//DServo.setDirectionPin(SERVO_ControlPin);

	//DServo.begin(SERVO_SET_Baudrate); // We now need to set Ardiuno to the new Baudrate speed
	//DServo.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE); // set mode to SERVO and set angle limits

	//ET.delay(500);

	//set to ALL for debugging purposes
	//DServo.setStatusPaket(SERVO_ID, ALL);
	//DServo.setStatusPaket(SERVO_RECEIVE_ID, ALL);

	//DServo.setID(SERVO_ID, SERVO_RECEIVE_ID);

	// Force the counter to be placed into memory
	volatile static int i = 0;
	// Enter an infinite loop, just incrementing a counter

	while (1) {
		int read_angle = desired_steering_angle;
		//DServo.servo(SERVO_ID, (desired_steering_angle + 180) * SCALING_FACTOR, 0x100); // Move servo to angle 0xA00(0xA00 * 0.088 degree) at speed 100
	}
	/*while (1) {
	 ET.delay(2000);
	 for (int j = 0; j < 500000; j++) {
	 }
	 DServo.servo(SERVO_ID, 0xA00, 0x100); // Move servo to angle 0xA00(0xA00 * 0.088 degree) at speed 100
	 ET.delay(2000);
	 for (int j = 0; j < 500000; j++) {
	 }
	 DServo.servo(SERVO_ID, 0x800, 0x100); // Move servo to angle 0x800(0x800 * 0.088 degree) at speed 100

	 int pos = DServo.readPosition(SERVO_RECEIVE_ID);

	 i++;
	 }*/

	DServo.deconstructor();
	return 0;
}
