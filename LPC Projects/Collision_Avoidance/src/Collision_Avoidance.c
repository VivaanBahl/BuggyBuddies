/*
===============================================================================
 Name        : Collision_Avoidance.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
#include "chip.h"
#include "board.h"

#define BAUDRATE 19200
//Speed of sound in m/s at 20C (68F)
#define SPEEDOFSOUND 343

#define TEST_CCAN_BAUD_RATE 1000000

/* Transmit and receive ring buffer sizes */
#define UART_RB_SIZE 64


enum ReadingType {LISTEN = 0, SHORT = 1, LONG = 2};

CCAN_MSG_OBJ_T msg_obj;

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];

static void Init_UART_PinMux(void)
{
#if (defined(BOARD_NXP_XPRESSO_11U14) || defined(BOARD_NGX_BLUEBOARD_11U24))
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_18 used for RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 19, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_19 used for TXD */
#elif (defined(BOARD_NXP_XPRESSO_11C24) || defined(BOARD_MCORE48_1125))
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
#else
#error "No Pin muxing defined for UART operation"
#endif
}

const char BURSTANDCAPTURELONG[] = {0x00, 0x55, 0x51, 0x02, 0x00};


/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART_IRQHandler(void)
{
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}

bool TestUart()
{
	//Send test command to ultrasonic
	Board_UARTPutChar(0x00);
	Board_UARTPutChar(0x55);
	Board_UARTPutChar(0x01);
	Board_UARTPutChar(0x00);

	//Board should respond to test command with 0x12 0x34 0xB9
	if (Board_UARTGetChar() != 0x12) return false;
	if (Board_UARTGetChar() != 0x34) return false;
	if (Board_UARTGetChar() != 0xB9) return false;
	return true;
}


bool UpdateEEPROM(uint8_t address, uint8_t data)
{
	Board_UARTPutChar(0x00);
	Board_UARTPutChar(0x55);
	Board_UARTPutChar(0x31);
	Board_UARTPutChar(address);
}


void ReadFIFO()
{
    Board_UARTPutChar(0x00);
    Board_UARTPutChar(0x55);
    Board_UARTPutChar(0x41);
    Board_UARTPutChar(0x00);
}


uint16_t BurstAndCapture(enum ReadingType type)
{
//    Chip_UART_SendRB(LPC_USART, &txring, BURSTANDCAPTURELONG, 5);

	Board_UARTPutChar(0x00);
    Board_UARTPutChar(0x55);
    Board_UARTPutChar(0x51);
    Board_UARTPutChar(0x02); //Hard coded long distance value
    Board_UARTPutChar(0x00);

    uint8_t high = 0;
    uint8_t low = 0;
    uint8_t checksum = 0;

    uint32_t i = 0;
    //Delay to allow ultrasonic to settle
	while (i < 100000)
	{
		i++;
	}

    high = Board_UARTGetChar();
    low = Board_UARTGetChar();
    checksum = Board_UARTGetChar();

    //TODO: Verify that the checksum matches the high and low byte

    uint16_t timeOfFlight = (((uint16_t)high << 8) + (uint16_t)low);

    return timeOfFlight;
}

double GetDistance()
{
    uint16_t timeOfFlight = 0;
    timeOfFlight = BurstAndCapture(LONG);
    if (timeOfFlight  == (uint16_t)-1) return 0;
    return ((double) timeOfFlight) * SPEEDOFSOUND/2000;

}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {
	//Board_UARTPutSTR("sent\n");
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occured on the CAN bus */
void CAN_error(uint32_t error_info) {
	//Board_UARTPutSTR("Error!\n");
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

int main(void)
{
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


    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();


	Board_Init();

	// Set the LED to the state of "On"
	Board_LED_Set(0, true);

	/* Setup UART for 19.2K8N1 */
	// The RX pin is pullup because UART by default (when plugged into usb) pulls up to 5V
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6,(IOCON_FUNC1 | IOCON_MODE_PULLUP));
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7,(IOCON_FUNC1 | IOCON_MODE_INACT));
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, BAUDRATE);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);
	/* Before using the ring buffers, initialize them using the ring
	buffer init function */
	//RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	//RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

	/* Enable receive data and line status interrupt */
	//Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	//NVIC_SetPriority(UART0_IRQn, 1);
	//NVIC_EnableIRQ(UART0_IRQn);

	//Setup CAN
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

    while(1)
    {
    	double distance = GetDistance();
    	uint32_t uint_distance = (uint32_t)distance;

    	msg_obj.msgobj  = 0;
    	msg_obj.mode_id = 0x000;
    	msg_obj.mask    = 0x0;
    	msg_obj.dlc     = 4;
    	msg_obj.data[0] = (uint8_t)(uint_distance & 0xFF);
    	msg_obj.data[1] = (uint8_t)((uint_distance >> 8) & 0xFF);
    	msg_obj.data[2] = (uint8_t)((uint_distance >> 16) & 0xFF);
    	msg_obj.data[3] = (uint8_t)((uint_distance >> 24) & 0xFF);
    	LPC_CCAN_API->can_transmit(&msg_obj);

    	volatile int i = 0;
    	while (i < 10000)
    	{
    		i++;
    	}
    }
    return 0 ;
}
