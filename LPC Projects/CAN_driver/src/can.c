/*
 * CAN driver that implements basic CAN_init, CAN_send, and CAN_receive
 * User has to write their own CAN_tx, CAN_rx, and CAN_error functions
 */

#include "board.h"

CCAN_MSG_OBJ_T* msg_obj;

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

void CAN_init(uint32_t baud_rate, CCAN_MSG_OBJ_T* msg_object,
		void (*CAN_rx)(uint8_t msg_obj_num), void (*CAN_tx)(uint8_t msg_obj_num),
		void (*CAN_error)(uint32_t error_info))
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
	baudrateCalculate(baud_rate, CanApiClkInitTable);
	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);
	msg_obj = msg_object;
}

/* Can only do uint64_t right now */
void CAN_send(uint8_t msgobj, uint8_t dlc, uint32_t mode_id, uint32_t mask, uint64_t value)
{
	msg_obj->msgobj  = msgobj;
	msg_obj->mode_id = mode_id;
	msg_obj->mask    = mask;
	msg_obj->dlc     = dlc;
	msg_obj->data[0] = value & 0xFF;
	msg_obj->data[1] = (value >> 8) & 0xFF;
	msg_obj->data[2] = (value >> 16) & 0xFF;
	msg_obj->data[3] = (value >> 24) & 0xFF;
	msg_obj->data[3] = (value >> 32) & 0xFF;
	msg_obj->data[3] = (value >> 40) & 0xFF;
	msg_obj->data[3] = (value >> 48) & 0xFF;
	msg_obj->data[3] = (value >> 56) & 0xFF;
	LPC_CCAN_API->can_transmit(msg_obj);
}

void CAN_receive(uint8_t msgobj, uint32_t mode_id, uint32_t mask)
{
	msg_obj->msgobj = msgobj;
	msg_obj->mode_id = mode_id;
	msg_obj->mask = mask;
	LPC_CCAN_API->config_rxmsgobj(msg_obj);
}


