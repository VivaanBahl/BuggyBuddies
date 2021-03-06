/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "board.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define LPC_SSP           LPC_SSP0

static Chip_SSP_DATA_SETUP_T xf_setup;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

static uint8_t status_tx_buf[64];
static uint8_t status_rx_buf[64];
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	// according to http://www.chlazza.net/sdcardinfo.html
	// sending over the status register is done by CMD13

	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
//		result = RAM_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
//		result = MMC_disk_status();

		// translate the reslut code here
		for (int i = 0; i < 64; i++)
		{
			status_tx_buf[i] = 0xFF;
		}

		status_tx_buf[0] = 0x40 | 13;
		status_tx_buf[1] = 0;
		status_tx_buf[2] = 0;
		status_tx_buf[3] = 0;
		status_tx_buf[4] = 0;
		status_tx_buf[5] = 0x1;

		xf_setup.length = 64;
		xf_setup.tx_data = status_tx_buf;
		xf_setup.rx_data = status_rx_buf;
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);

		return 0;

	case DEV_USB :
//		result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

static uint8_t init_tx_buf[0x100];
static uint8_t init_rx_buf[0x100];
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;
	char cmd_header;
	uint32_t argument;

	switch (pdrv) {
	case DEV_RAM :
//		result = RAM_disk_initialize();

		// translate the reslut code here

		return stat;

	case DEV_MMC :

		// Toggle SCK at least 74 times, with MOSI and SS held high

		// send down a CMD0
		cmd_header = 0x40;
		argument = 0x0;
		char crc = 0x95;

		for (int i = 0; i < 0x100; i++)
		{
			init_tx_buf[i] = 0xFF;
		}

		xf_setup.length = 0x100;
		xf_setup.tx_data = init_tx_buf;
		xf_setup.rx_data = init_rx_buf;
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);

		// http://elm-chan.org/docs/mmc/im/sdinit.png
		// flowchart for SD init
		// we just choose to send cmd1 3 times because vivaan is dumb lol

		init_tx_buf[0] = cmd_header;
		init_tx_buf[1] = 0;
		init_tx_buf[2] = 0;
		init_tx_buf[3] = 0;
		init_tx_buf[4] = 0;
		init_tx_buf[5] = crc;

		init_tx_buf[10] = 0x40 | 8;
		init_tx_buf[11] = 0;
		init_tx_buf[12] = 0;
		init_tx_buf[13] = 0x1;
		init_tx_buf[14] = 0xAA;
		init_tx_buf[15] = 0x87;

		init_tx_buf[20] = 0x40 | 55;
		init_tx_buf[21] = 0;
		init_tx_buf[22] = 0;
		init_tx_buf[23] = 0;
		init_tx_buf[24] = 0;
		init_tx_buf[25] = 0;

		init_tx_buf[30] = 0x40 | 1;
		init_tx_buf[31] = 0x40;
		init_tx_buf[32] = 0;
		init_tx_buf[33] = 0;
		init_tx_buf[34] = 0;
		init_tx_buf[35] = 0;

		init_tx_buf[40] = 0x40 | 1;
		init_tx_buf[41] = 0x40;
		init_tx_buf[42] = 0;
		init_tx_buf[43] = 0;
		init_tx_buf[44] = 0;
		init_tx_buf[45] = 0;

		init_tx_buf[50] = 0x40 | 1;
		init_tx_buf[51] = 0x40;
		init_tx_buf[52] = 0;
		init_tx_buf[53] = 0;
		init_tx_buf[54] = 0;
		init_tx_buf[55] = 0;

		init_tx_buf[60] = 0x40 | 58;
		init_tx_buf[61] = 0;
		init_tx_buf[62] = 0;
		init_tx_buf[63] = 0;
		init_tx_buf[64] = 0;
		init_tx_buf[65] = 0;

		init_tx_buf[60] = 0x40 | 16;
		init_tx_buf[61] = 0;
		init_tx_buf[62] = 0;
		init_tx_buf[63] = 0x2;
		init_tx_buf[64] = 0;
		init_tx_buf[65] = 0;

		Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC0 | IOCON_MODE_INACT));	/* SSEL0 */
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 2);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, 0, 2);

		xf_setup.length = 0x100;
		xf_setup.tx_data = init_tx_buf;
		xf_setup.rx_data = init_rx_buf;
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);
//		Chip_SSP_WriteFrames_Blocking(LPC_SSP, tx_buf, 0x100);
//		Chip_SSP_ReadFrames_Blocking(LPC_SSP, rx_buf, 1);

		Chip_GPIO_SetPinOutHigh(LPC_GPIO, 0, 2);
		Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SSEL0 */

		stat = RES_OK;
		// translate the reslut code here

		return stat;

	case DEV_USB :
//		result = USB_disk_initialize();

		// translate the reslut code here
//		i++;

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

static uint8_t read_tx_buf[3000];
static uint8_t read_rx_buf[3000];
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

//		result = RAM_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

//		result = MMC_disk_read(buff, sector, count);
		Board_UARTPutSTR("diskread\n");

		for (int i = 0; i < 3000; i++) {
			read_tx_buf[i] = 0xFF;
		}

		read_tx_buf[0] = 0x40 | 17;
		read_tx_buf[1] = (sector >> 24) & 0xFF;
		read_tx_buf[2] = (sector >> 16) & 0xFF;
		read_tx_buf[3] = (sector >> 8) & 0xFF;
		read_tx_buf[4] = (sector) & 0xFF;
		read_tx_buf[5] = 0x1;

		xf_setup.length = 3000;
		xf_setup.tx_data = read_tx_buf;
		xf_setup.rx_data = read_rx_buf;
		xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

		Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);

		for (int i = 0; i < 3000; i++) {
			if (read_rx_buf[i-1] == 0xFE) {
				for (int j = 0; j < 512; j++) {
					buff[j] = read_rx_buf[j + i];
				}
				break;
			}
		}
		
		// TODO fix so that we can write more than 4 sectors

		// translate the reslut code here

		return 0;

	case DEV_USB :
		// translate the arguments here

//		result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

//		result = RAM_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

//		result = MMC_disk_write(buff, sector, count);
		Board_UARTPutSTR("diskwrite\n");

		// translate the reslut code here

		return res;

	case DEV_USB :
		// translate the arguments here

//		result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;

	case DEV_MMC :

		// Process of the command for the MMC/SD card
		Board_UARTPutSTR("ioctl\n");

		return res;

	case DEV_USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;
}

