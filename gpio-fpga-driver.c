/************************************************************************
*	Author: Mitchell Orsucci
*	
*	This software is offered freely by Digilent Inc.
*
*	Creation Date: July 24, 2017
*	
*	To be used in tandem with a UIO driver and the Xilinx AXI GPIO
*	that has been implemented on an FPGA
*	
*	Use at your own risk and peril
*
************************************************************************/ 
#include <stdio.h>
#include <stdlib.h>
#include "uio-user.h"
#include "gpio-fpga.h"

GPIO GPIO_init(uint8_t uioNum, uint8_t mapNum) {
	if(uioNum < 0 || mapNum < 0) {
		printf("That is not a valid UIO device or map number\n");
		printf("Check /sys/class/uio for more information about"); 
		printf(" the available UIO devices\n");
		exit(EXIT_FAILURE);
	}
	void * vm;
	UIO * uio = UIO_MAP(uioNum, mapNum);
	vm = uio->mapPtr;
	
	/* Initialize both channels to output with value 0 */
	setChannelDirection(vm, 0x00000000, 1);
	setChannelDirection(vm, 0x00000000, 2);
	setChannelValue(vm, 0, 1);
	setChannelValue(vm, 0, 2);
 	
 	return vm;
}

int8_t setPinMode(GPIO vm, int8_t pinNum, int8_t direction) {
	if(pinNum > CHANNEL_MAX_SIZE || pinNum < CHANNEL_MIN_SIZE || direction > INPUT || direction < OUTPUT) {
		printf("Invalid Argument passed to setPinMode()\n");
		printf("Please look at the valid function arguments\n");
		return -1;
	}

	/* Get the current settings for direction */
	uint32_t currentDir = ACCESS_REG(vm, CHANNEL_1_DIRECTION);

	/* Clear the bit in question */
	currentDir &= ~(1 << (pinNum - 1));
	currentDir |= (direction << (pinNum - 1));

	/* Set the new configuration */
	ACCESS_REG(vm, CHANNEL_1_DIRECTION) = currentDir;
	return 0;
}

int8_t digitalWrite(GPIO vm, int8_t pinNum, int8_t value) {
	if(pinNum > CHANNEL_MAX_SIZE || pinNum < CHANNEL_MIN_SIZE || value < LOW) {
		printf("Invalid Argument passed to digitalWrite()\n");
		printf("Please look at the valid function arguments\n");
		return -1;
	}
	value = !!value; /* convert any thing over 0 to a 1 */

	/* Get the current settings for value */
	uint32_t currentVal = ACCESS_REG(vm, CHANNEL_1_DATA);
		/* Clear the bit in question */
	currentVal &= ~(1 << (pinNum - 1));
	currentVal |= (value << (pinNum - 1));
	
	ACCESS_REG(vm, CHANNEL_1_DATA) = currentVal;
	return 0;
}

int8_t digitalRead(GPIO vm, int8_t pinNum) {
	if(pinNum > CHANNEL_MAX_SIZE || pinNum < CHANNEL_MIN_SIZE) {
		printf("Invalid Argument passed to digitalRead()\n");
		printf("Please look at the valid function arguments\n");
		return -1;
	}

	return ((ACCESS_REG(vm, CHANNEL_1_DATA) >> (pinNum - 1)) & 0x01);
}

int8_t setChannelValue(GPIO vm, uint32_t valMask, int8_t channel){
	if(channel > 2 || channel < 1) {
		printf("Invalid Argument passed to setChannelValue()\n");
		printf("Please look at the valid function arguments.\n");
		return -1;
	}

	ACCESS_REG(vm, CHANNEL_1_DATA + ((channel - 1) * 8)) = valMask;
	return 0;
}

uint32_t readChannelValue(GPIO vm, int8_t channel) {
	if(channel > 2 || channel < 1) {
		printf("Invalid Argument passed to readChannelValue()\n");
		printf("Please look at the valid function arguments.\n");
		return -1;
	}

	return ACCESS_REG(vm, CHANNEL_1_DATA + ((channel - 1) * 8));
}

int8_t setChannelDirection(GPIO vm, uint32_t dirMask, int8_t channel) {
	if(channel > 2 || channel < 1) {
		printf("Invalid Argument passed to setChannelDirection()\n");
		printf("Please look at the valid function arguments.\n");
		return -1;
	}
	ACCESS_REG(vm, CHANNEL_1_DIRECTION + ((channel - 1) * 8)) = dirMask;
	return 0;
}

uint8_t GPIO_Close(GPIO vm) {
	return UIO_UNMAP(vm);
}