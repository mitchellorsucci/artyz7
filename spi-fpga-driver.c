/************************************************************************
*	Author: Mitchell Orsucci
*	
*	This software is offered freely by Digilent Inc.
*
*	Creation Date: July 24, 2017
*	
*	To be used in tandem with a UIO driver and the Xilinx AXI QUAD SPI IP
*	that has been implemented on an FPGA
*	
*	Use at your own risk and peril
*
************************************************************************/ 
#include <stdio.h>
#include <stdlib.h>
#include "uio-user.h"
#include "spi-fpga.h"


SPI SPI_init(uint8_t uioNum, uint8_t mapNum) {
	if(uioNum < 0 || mapNum < 0) {
		printf("That is not a valid UIO device or map number\n");
		printf("Check /sys/class/uio for more information about"); 
		printf(" the available UIO devices\n");
		exit(EXIT_FAILURE);
	}
	void * vm;
	UIO * uio = UIO_MAP(uioNum, mapNum);
	vm = uio->mapPtr;
	ACCESS_REG(vm, SPI_SRR) = SPI_RST; // Reset the hardware
	usleep(1000);
	
	/* Configure the SPI to begin in mode CPOL = 0, CPHA = 0
		User can change the SPI mode by using SPI_setMode() */
	ACCESS_REG(vm, SPI_CR) = (CR_MAN_SLAVE_SEL | CR_RST_RX | CR_RST_TX | CR_MSTR_MODE); // Set controls for the device
	ACCESS_REG(vm, SPI_GLB_INT) = 0x00000000; // Disable global interrupt
	ACCESS_REG(vm, SPI_SS) = NO_SLAVES_SELECTED; /* Select Slave*/
	ACCESS_REG(vm, SPI_IIER) = INT_TX_EMPTY;
 	return vm;
}

void SPI_setMode(SPI vm, byte CPOL, byte CPHA) {
	uint32_t config_reg = ACCESS_REG(vm, SPI_CR);

	/* Clears the current CPOL and CPHA settings */
	config_reg &= ~(CR_CPHA | CR_CPOL);

	/* Adds in the new settings and writes them to the config register */
	config_reg |= ((!!CPOL) << 3);
	config_reg |= ((!!CPHA) << 4);
	ACCESS_REG(vm, SPI_CR) = config_reg;
}

void SPI_Transfer(SPI vm, byte * tx_buffer, byte * rx_buffer, byte numBytes) {
	if(rx_buffer == NULL) {
		byte dummy[numBytes];
		rx_buffer = dummy;
	}
	byte tx_byteCount = numBytes;
	byte rx_byteCount = numBytes;
	ACCESS_REG(vm, SPI_SS) = SLAVE_1;
	
	/* Fill the TX FIFO with numBytes from tx_buffer */
	while(tx_byteCount > 0) {
		ACCESS_REG(vm, SPI_DTX) = *tx_buffer++;
		tx_byteCount--;
	}

	/* Enable the hardware, so as to put the data on the bus */
	ACCESS_REG(vm, SPI_CR) |= CR_SPI_ENABLE;

	while(!(ACCESS_REG(vm, SPI_IISR) & INT_TX_EMPTY)) {
		/* Wait til the TX FIFO is empty */
		/* Then we know all data has been transferred */
	}

	ACCESS_REG(vm, SPI_SS) = NO_SLAVES_SELECTED; /* Select no slaves */
	ACCESS_REG(vm, SPI_CR) &= ~CR_SPI_ENABLE;	/* Disable the hardware from making further transfers */
	ACCESS_REG(vm, SPI_IISR) |= INT_TX_EMPTY;	/* Reset the TX Empty interrupt */

	while(rx_byteCount > 0) {
		*rx_buffer++ = ACCESS_REG(vm, SPI_DRX);
		rx_byteCount--;
	}

}

uint8_t SPI_Close(SPI vm) {
	return UIO_UNMAP(vm);
}