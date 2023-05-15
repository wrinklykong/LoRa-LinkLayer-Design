// my beautiful network!! ^__^
// CURRENTLY HOLDS THE NODE DATA!

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include <bcm2835.h>
#include "Loranga.h"
#include "LoRa.h"
#include "SX1272-Loranga.h"
#include "Packet.h"

#define GETOPMODE readReg(REG_OP_MODE)
#define WRITEOPMODE(X) (writeReg(REG_OP_MODE, X))
#define GETTIME(X) (clock_gettime(CLOCK_REALTIME, X))

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

uint8_t spi_data[128];
uint8_t num_bytes_recv, modem_status, pkt_payload[64];
uint8_t *unp, pos;
uint32_t num_valid_hdrs, num_valid_pkts;
int count = -1;
/* Relates to LoRa Config */
uint8_t val, header, optimize, threshold;
int preamble_length = 8;                // Typical preamble length of 8
int payload_size = 8;
int i, pkt_rssi, rssi, pkt_snr = 0;
deviceMode *DEVICE_MODE;
long nanosecs;
struct timespec now,then;


/* Sets registers and interal things to receive data on the FIFO */
void setupFIFOReceive() {
        // try to receive count frames
        writeReg(REG_DIO_MAPPING1, DIO0_RX_DONE);
        writeReg(REG_MAX_PAYLOAD_LENGTH, 0xff);
        // fifo pointers
        writeReg(REG_FIFO_ADDR_PTR, 0x0);
        writeReg(REG_FIFO_RX_BASE_ADDR, 0x0);
        // set RX FS mode
        WRITEOPMODE ( LORA_FSRX_MODE );
        while ( GETOPMODE != LORA_FSRX_MODE );          // waits until FS mode is active
        WRITEOPMODE ( LORA_RX_MODE );                   // receive mode
        while(!((val=readReg(REG_IRQ_FLAGS))&(IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERR))) {
                usleep(1000);
                //printf("Payload size = %d valid_hdrs = %d valid_pkts = %d modem status %2x\n",
                        //num_bytes_recv, num_valid_hdrs, num_valid_pkts, modem_status);
        }
        // set all flags?
        writeReg(REG_IRQ_FLAGS, 0xFF);
        //spi_data[512];
        memset(spi_data, 0, 512);
}

/* Sets registers and interal things to send data on the FIFO */
void setupFIFOSend() {
        printf("The preamble length is %d + 4.25 symbols\n", (int) readReg(REG_PREAMBLE_LSB_LORA));
        printf("The default payload length is %2x (1 is correcT)", readReg(REG_PAYLOAD_LENGTH_LORA));

        writeReg(REG_DIO_MAPPING1, DIO0_TX_DONE);       /* called in case use_dio0 is TRUE ? */
        // we are only going to send ONE time
        writeReg(REG_PA_CONFIG, readReg(REG_PA_CONFIG) | PA_BOOST);
        // Standby to access the FIFO
        WRITEOPMODE ( LORA_STANDBY_MODE );
        // set FIFI pointers
        writeReg(REG_FIFO_ADDR_PTR, 0x0);
        writeReg(REG_FIFO_TX_BASE_ADDR, 0x0);
}

void sendPacket( Packet p ) {
        /* Initialize the FIFO for Sending */
        setupFIFOSend();

        /* Load packet into fifo */
        uint8_t packetDecoded[] = decodePacket(p);
        memcpy((void*)(pkt_payload+8),(void*)packetDecoded,64); // i hope this works lmao

        /* Send the data through FIFO */
        writeReg(REG_PAYLOAD_LENGTH_LORA, (uint8_t) payload_size);      // for some reason Hoperf SE says this is only needed for implicit header ??
        // load payload in FIFO
        writeNReg(REG_FIFO, payload_size, pkt_payload);
        // TRANSMIT MODE TO SEND DATA!!! MAUHAHAHA
        WRITEOPMODE ( LORA_TX_MODE );
        GETTIME(&then);
        // waiting for register to be done with TX done
        while (!((val = readReg(REG_IRQ_FLAGS)) & IRQ_TX_DONE)) {
                i++;
                usleep(100);            // delay between SPI reads
        }
        // moves to STANDBY after sending
        GETTIME(&now);
        nanosecs = DIFF(now,then);
        printf("Final IRQ bits = %2x Final OP MODE = %2x loop count = %d nsecs = %ld\n", readReg(REG_IRQ_FLAGS), GETOPMODE, i, nanosecs);
        writeReg(REG_IRQ_FLAGS, 0xFF);  // Clears the IRQs
        usleep(10000);
}

void sendData() {
        printf("The preamble length is %d + 4.25 symbols\n", (int) readReg(REG_PREAMBLE_LSB_LORA));
        printf("The default payload length is %2x (1 is correcT)", readReg(REG_PAYLOAD_LENGTH_LORA));
        setupFIFOSend();

        /*Build payload, replace with whatever method you choose */
        payload_size = 8;                       // typical payload size ? 8 somethings
        printf("Initial payload size = %d\n", payload_size);
        memcpy((void*)(pkt_payload+payload_size+1), "Header:1234",12);
        payload_size += 12;
        memcpy((void*)(pkt_payload+payload_size+1), "Hello!", 7);
        payload_size += 7;
        char hash = simpleHashValue(pkt_payload+9, payload_size-9);
        memcpy((void*)(pkt_payload+payload_size+1),
                        &hash, 1);
        payload_size += 1;
        char eof = 0xFF;
        memcpy((void*)(pkt_payload+payload_size+1), &eof, 1);           // end of frame thingy
        payload_size += 1;

        /* Send the data */
        writeReg(REG_PAYLOAD_LENGTH_LORA, (uint8_t) payload_size);      // for some reason Hoperf SE says this is only needed for implicit header ??
        // load payload in FIFO
        writeNReg(REG_FIFO, payload_size, pkt_payload);
        // TRANSMIT MODE TO SEND DATA!!! MAUHAHAHA
        WRITEOPMODE ( LORA_TX_MODE );
        GETTIME(&then);
        // waiting for register to be done with TX done
        while (!((val = readReg(REG_IRQ_FLAGS)) & IRQ_TX_DONE)) {
                i++;
                usleep(100);            // delay between SPI reads
        }
        // moves to STANDBY after sending
        GETTIME(&now);
        nanosecs = DIFF(now,then);
        printf("Final IRQ bits = %2x Final OP MODE = %2x loop count = %d nsecs = %ld\n", readReg(REG_IRQ_FLAGS), GETOPMODE, i, nanosecs);
        writeReg(REG_IRQ_FLAGS, 0xFF);  // Clears the IRQs
        usleep(10000);
}

Packet receivePacket() {
        setupFIFOReceive();
        if (!(val & IRQ_RX_TIMEOUT)) {
                // looks good, grab the frame size and other counters
                unp = readNReg(REG_RX_NB_BYTES, 6, spi_data);
                num_bytes_recv = *unp;
                num_valid_hdrs = (unp[1]<<8) | unp[2];
                num_valid_pkts = (unp[3]<<8) | unp[4];
                modem_status = *(unp+5);
                // get the payload !
                writeReg(REG_FIFO_ADDR_PTR, pos = readReg(REG_FIFO_RX_CURRENT_ADDR));
                unp = readNReg(REG_FIFO, num_bytes_recv, spi_data);

                /* Parse the packet! */
                Packet received = convertToPacket(spi_data);

                /* Verify the hash of the packet to see if was sent correctly */
                if ( hashPacket(received) == received.hash ) {
                        printPacket(received);
                        return received;
                }
                // idk?
                // unp = readNReg(REG_PKT_SNR_VALUE,3,spi_data);
                // pkt_snr = (*unp) >> 2;
                // pkt_rssi = -137 + *(unp+2);
                // rssi = -137 + *(unp+2);
                // return received;
                // printf("Payload size = %d valid_hdrs = %d valid_pkts = %d modem status %2x irq = %2x\n",
                //         num_bytes_recv,num_valid_hdrs,num_valid_pkts,modem_status,val);
        }
        return NULL;

}

int testLoraTransceiver() {
        // resets the LORA chip
        bcm2835_gpio_clr(LORA_RST);
        usleep(500);                    /* wait before waking up again */
        bcm2835_gpio_set(LORA_RST);
        usleep(500);                    /* turn back on the device */
        printf("LoRa chip version = %x\n", readReg(REG_VERSION));
        printf("Any non-zero value != 12 should be investigated, zero value indicates an issue\n");

        uint64_t then_stc = GETSTC;
        uint64_t now_stc = GETSTC;
        uint64_t time_stc;
        uint64_t gettime;
        uint64_t write_stc;                     // time it takes to write to REG
        uint64_t read_stc;                      // time it takes to read REG

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

/* Generates packets for communication between node that does not carry data */
Packet generateCommunicationPacket( uint8_t packetType, uint8_t srcID, uint8_t dstID, uint8_t srcPrefix, uint8_t dstPrefix ) {
        if ( packetType != 0x05 && packetType != 0x06 ) {
                return createPacket(packetType, srcID, dstID, srcPrefix, dstPrefix, 3, 0, "NA");
        }
        printf("ERROR: Incorrect Packet passed into function, please check arguments.\n");
        return NULL;
}

/* Generates packets for B2B and N2N communications */
Packet generateDataPacket( uint8_t packetType, uint8_t srcID, uint8_t dstID, uint8_t srcPrefix, uint8_t dstPrefix, uint8_t msg, uint8_t payload[] ) {
        if ( packetType == 0x05 | packetType == 0x06 ) {
                return createPacket(packetType, srcID, dstID, srcPrefix, dstPrefix, 48, msg, payload);
        }
        printf("ERROR: Incorrect Packet passed into function, please check arguments.\n");
        return NULL;
}

void testPacketSendingAndReceiving() {
        Packet beaconAdvert = generateCommunicationPacket(  );
        printPacket(beaconAdvert);
        sendPacket(beaconAdvert);
        Packet response = receivePacket();
        if ( response != NULL ) {
                printPacket(response);
        }
}

void beaconMainLoop() {
        /* First, send out a BEACON_ADVERT packet*/
        while (true) {
                Packet beaconAdvert = generateDataPacket();
                sendPacket(beaconAdvert);
                Packet response = receivePacket();
                /* Wait about 3 seconds, ping nearby beacons for their prefixes */
                while ( initialTimeNotCompleted ) {
                        Packet response = receivePacket();
                        if ( response != NULL ) {
                                if ( response.packetType == BEACON_RESPONSE ) {
                                        // store that beacon prefix        
                                }
                        }
                }
                // generate a new prefix not taken, set it to the system
                // Main loop
                while ( true ) {
                        Packet response = receivePacket();              // should implement a timer if nothing found
                        switch ( response.packetType ) {
                                case NODE_ADVERT:
                                        /* Send a BEACON_PURCH with the prefix and the ID given */
                                        break;
                                case BEACON_2_BEACON:
                                        /* Send back the contents out into the userbase */
                                        break;
                                case BEACON_ADVERT:
                                        /* send a BEACON_RESPONSE with Prefix */
                                        break;
                                case CONNECTION_CHECK:
                                        /* Check the ID and see if its in network, reset the timer for activity */
                                        break;
                                default:
                                        printf("Unknown packet received.\n");
                                        break;
                        }
                        // check for activity on devices, if one has not been active for X amount, ping it
                        /* Send CONNECTION_CHECK */
                }
        }
}

void nodeMainLoop() {
        Packet nodeAdvert = generateDataPacket(NODE_ADVERT);
        sendPacket(beaconAdvert);
        Packet response;
        while (true) {
                response = receivePacket();
                if ( response.packetType == BEACON_PURCH ) {
                        // set my id and prefix to those sent
                        break;          // break out of loop
                }
        }
        /* Main communication loop */
        while (true) {
                if ( dontNeedToTransmit ) {
                        response = receivePacket();
                }
        }
        /* Implement some sort of interrupt? or way to change when I need to use the system */
}

int main( int argc, char* argv[] )
{
        if ( argc < 2 ) {
                printf("ERROR: Not enough arguments!\nUSAGE: a.exe (B)EACON/(N)ODE\n");
                return -1;
        }
        DEVICE_MODE = (strcmp(argv[1], "B")) ? &(networkDeviceModes[0]) : &(networkDeviceModes[1]);
        printf("Current Mode: %s\n", DEVICE_MODE->name);
        int lockId = -1;                                // holds the FD of the _loranga_lock_ file for use in unlock later on


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
        bcm2835_spi_begin();                                    // begins SPI
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
        val             = 0;
        header          = HEADER_OFF;
        optimize        = OPTIMIZE_SF6;
        threshold       = THRESHOLD_SF6;
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
                        testPacketSendingAndReceiving();
                        break;
                case 'B':
                        printf("We are in BEACON mode!\n");
                        testPacketSendingAndReceiving();
                        break;
                default:
                        printf("UNKNOWN MODE! what the H!\n");
                        break;
        }





        bcm2835_spi_end();                                      // ends SPI
        bcm2835_close();                                        // ends bcm2835 stuff,  do this LAST to prevent memory leaks
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
