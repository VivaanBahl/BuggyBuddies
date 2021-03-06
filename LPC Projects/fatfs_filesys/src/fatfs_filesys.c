/*
===============================================================================
 Name        : fatfs_filesys.c
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
#include "/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Libraries/ff13/source/ff.h"


#define SSP_MODE_TEST       1	/*1: Master, 0: Slave */
#define LPC_SSP           LPC_SSP0
#define SSP_IRQ           SSP0_IRQn
#define SSPIRQHANDLER     SSP0_IRQHandler
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define IOCON_PIN_ID				IOCON_PIO0_3 /* IOCON pin identifer */

// TODO: insert other include files here
static SSP_ConfigFormat ssp_format;

// TODO: insert other definitions and declarations here
void init_spi_pins()
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* UART RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* UART TXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MISO0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MOSI0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SSEL0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO2_11, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SCK0 */
	Chip_IOCON_PinLocSel(LPC_IOCON, IOCON_SCKLOC_PIO2_11);
}

int main(void) {

	/* SSP initialization */
	SystemCoreClockUpdate();
	Board_Init();
	Chip_GPIO_Init(LPC_GPIO);
	Chip_UART_Init(LPC_USART);
	init_spi_pins();
	Board_LED_Set(0, false);
	Chip_SSP_Init(LPC_SSP);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_DATA_BITS;
	ssp_format.clockMode = SSP_CLOCK_MODE0;
	Chip_SSP_SetFormat(LPC_SSP, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);
	Chip_SSP_SetMaster(LPC_SSP, SSP_MODE_TEST);
	Chip_SSP_Enable(LPC_SSP);

	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	Board_UARTPutSTR("hello world\n");

	// TODO: insert code here
	FATFS *fs = malloc(sizeof(FATFS));
	f_mount(fs, "1:", 0);

	FIL *fp = malloc(sizeof(FIL));
	if(f_open(fp, "1:log1.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS) == FR_OK) {
		char buf[10];
		int j;
		if (f_read(fp, buf, 16, &j) == FR_OK) {
			Board_UARTPutSTR(buf);
			Board_UARTPutSTR("\n");
			f_close(fp);
		}
	}

	// Force the counter to be placed into memory
	volatile static int i = 0 ;
	// Enter an infinite loop, just incrementing a counter
	while(1) {
		i++ ;
	}
	return 0 ;
}
