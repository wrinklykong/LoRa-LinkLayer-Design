// my beautiful network!! ^__^

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <bcm2835.h>
#include "Loranga.h"
#include "LoRa.h"
#include "SX1272-Loranga.h"

#define GETOPMODE readReg(REG_OP_MODE)
#define WRITEOPMODE(X) (writeReg(REG_OP_MODE,X))

/* Channel = 903.08Mhz */
#define CHANNEL CH_00_900
/* BANDWIDTH: 500kHz */
#define BANDWIDTH BW_500
/* CODING RATE: 4/5 */
#define CODING_RATE CR_5
/* SPREADING FACTOR: 64 */
#define SPREADING_FACTOR SF_6

typedef struct deviceMode {
	char type;
	char* name;
} deviceMode;
deviceMode networkDeviceModes[] = { {'N', "Node"}, {'B', "Beacon"} };

uint8_t spi_data[512];
/* Relates to LoRa Config */
uint8_t val, header, optimize, threshold;
int preamble_length = 8;		// Typical preamble length of 8
deviceMode *DEVICE_MODE;

int testLoraTransceiver() {
	// resets the LORA chip
	bcm2835_gpio_clr(LORA_RST);
	usleep(500);			/* wait before waking up again */
	bcm2835_gpio_set(LORA_RST);
	usleep(500);			/* turn back on the device */
	printf("LoRa chip version = %x\n", readReg(REG_VERSION));
	printf("Any non-zero value != 12 should be investigated, zero value indicates an issue\n");

	uint64_t then_stc = GETSTC;
	uint64_t now_stc = GETSTC;
	uint64_t time_stc;
	uint64_t gettime;
	uint64_t write_stc;			// time it takes to write to REG
	uint64_t read_stc;			// time it takes to read REG

	gettime = time_stc = now_stc - then_stc;
	printf("The time to just get the time is %lld usecs\n\n", time_stc);

	printf("We will now find the times to change between LoRa states\n");
	printf("Set LoRa sleep mode\n");
	then_stc = GETSTC;
	// writes 1 to sleep mode, makes LoRa sleep
	WRITEOPMODE(LORA_SLEEP_MODE);
	now_stc = GETSTC;
	write_stc = ( now_stc - then_stc ) - gettime;
	then_stc = GETSTC;
	GETOPMODE;
	now_stc = GETSTC;
	read_stc = ( now_stc - then_stc ) - gettime;
	printf("Time for read: %lld usecs\n\n", write_stc);
	printf("Time for write: %lld usecs\n\n", read_stc);
	return 0;
}

int main( int argc, char* argv[] )
{
	if ( argc < 2 ) {
		printf("ERROR: Not enough arguments!\nUSAGE: a.exe (B)EACON/(N)ODE\n");
		return -1;
	}
	DEVICE_MODE = (strcmp(argv[1], "B")) ? &(networkDeviceModes[0]) : &(networkDeviceModes[1]);
	printf("Current Mode: %s\n", DEVICE_MODE->name);
	int lockId = -1;				// holds the FD of the _loranga_lock_ file for use in unlock later on
	

	// Begin LoRa, make sure we have access to LoRa currently
	lockId = loranga_lock();
	if ( lockId != -1 ) {
		printf("Successfully locked! We have control now\n");
	}
	else {
		printf("Failed to open file\n");
		return -1;
	}

	// Initialize BCM library and init setup
	// I am using the default values that were found in LorangaCheck.c
	if ( !bcm2835_init() ) {
		printf("BCM Init failed. Are you running as ROOT?\n");
		return -1;
	}
	bcm2835_spi_begin();					// begins SPI
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
	// removed lines about LED's I do not have that functionality here lol
	// Function select, setting these pins (First arg) in different modes (second, OUTP=output, INPT=input etc. see documentation)
	bcm2835_gpio_fsel(SIM800_ON_OFF, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(LORA_RST, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(QUECTEL_ON_OFF, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(QUECTEL_STATUS, BCM2835_GPIO_FSEL_INPT);

	// test receiver of LoRa
	// testLoraTransceiver();

	/* Sets the Channel frequency of the LoRa chip, puts first 3 bytes into spi_data */
	writeNReg(REG_FRF_MSB, 3, load3bytes(spi_data, CHANNEL));
	/* Populates these values with the SF6 (64?) settings, change if using other SF */
	val 		= 0;
	header 		= HEADER_OFF;
	optimize 	= OPTIMIZE_SF6;
	threshold 	= THRESHOLD_SF6;
	writeReg(REG_MODEM_CONFIG1, 
			putbits(putbits(putbits(val,header,0,0),CODING_RATE,3,1),BANDWIDTH,7,4));
	/* Optimizations for SF6 */
	writeReg(REG_DETECT_OPTIMIZE, optimize);
	writeReg(REG_DETECTION_THRESHOLD, threshold);
	/* RX Timeouts */
	writeReg(REG_MODEM_CONFIG2,
			putbits(putbits(SPREADING_FACTOR << 4, MAX_RX_TIMEOUT,1,0),CRC_ON,2,2));
	writeReg(REG_SYMB_TIMEOUT_LSB, MAX_RX_TIMEOUT);
	writeReg(REG_MODEM_CONFIG3, LOW_DATA_RATE_OPTIMIZE | AGC_AUTO_ON);
	/* Set preamble length (8 by default) */
	writeNReg(REG_PREAMBLE_MSB_LORA,2,load2bytes(spi_data, preamble_length));
	
	/* The main transmission */
	switch ( DEVICE_MODE->type ) {
		case 'N':
			printf("We are in NODE mode!\n");
			break;
		case 'B':
			printf("We are in BEACON mode!\n");
			break;
		default:
			printf("UNKNOWN MODE! what the H!\n");
			break;
	}





	bcm2835_spi_end();					// ends SPI
	bcm2835_close();					// ends bcm2835 stuff,  do this LAST to prevent memory leaks
	// End access to LoRa to avoid things from overlapping
	if ( loranga_unlock(lockId) != -1 ) {
		printf("Successfully unlocked! Loranga free to use\n");
	}
	else {
		printf("Failed to unlock file :,(");
		return -1;
	}

	return -1;
}
