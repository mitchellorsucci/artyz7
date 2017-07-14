#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h> //for uint8_t
#include <errno.h>
#include <string.h>

#define MAP_SIZE 0x11000
#define SPI_BASE 0x40010000
#define UIO_BASE 0x40001000
#define SPI_OFFSET SPI_BASE - UIO_BASE

/*******************************
 * SPI HARDWARE REGISTER OFFSETS
 ********************************/
#define SPI_GLB_INT 	0x1c /* Global Intr Enable Reg */
#define SPI_IISR 	0x20 /* Interrupt Status Reg */
#define SPI_IIER 	0x28 /* Interrupt Enable Reg */
#define SPI_SRR 	0x40 /* Software Reset Register */
#define SPI_CR 		0x60 /* Control Register */
#define SPI_SR 		0x64 /* Status Register */
#define SPI_DTX 	0x68 /* Data Transmit Register */
#define SPI_DRX 	0x6c /* Data Receive Register */
#define SPI_SS 		0x70 /* Slave Select Register */
#define SPI_TFO 	0x74 /* Tx Fifo Occupancy */
#define SPI_RFO 	0x78 /* Rx Fifo Occupancy */

/**********************************
 * PMOD GYRO Register Locations
 * ********************************/
#define WHO_AM_I 	0x0F
#define CTRL_REG1 	0x20
#define CTRL_REG2 	0x21
#define CTRL_REG3 	0x22
#define CTRL_REG4 	0x23
#define CTRL_REG5 	0x24
#define REFERENCE 	0x25
#define OUT_TEMP 	0x26
#define STATUS_REG 	0x27
#define OUT_X_L 	0x28
#define OUT_X_H 	0x29
#define OUT_Y_L 	0x2A
#define OUT_Y_H 	0x2B
#define OUT_Z_L 	0x2C
#define OUT_Z_H 	0x2D
#define FIFO_CTRL_REG 	0x2E
#define FIFO_SRC_REG 	0x2F
#define INT1_CFG 	0x30
#define INT1_SRC 	0x31
#define INT1_TSH_XH 	0x32
#define INT1_TSH_XL 	0x33
#define INT1_TSH_YH 	0x34
#define INT1_TSH_YL 	0x35
#define INT1_TSH_ZH 	0x36
#define INT1_TSH_ZL 	0x37
#define INT1_DURATION 	0x38

#define INT_TX_EMPTY 	0x00000004


#define RST_RX			0x00000040
#define RST_TX			0x00000020
#define MSTR_INHIBIT 	0x100
#define RX_EMPTY 	0x00000001
#define RX_FULL 	0x00000002
#define TX_EMPTY 	0x00000004
#define SPI_ENABLE	0x00000002

#define READ 		0x80
#define INCR_REG 	0x40

#define ACCESS_REG(OFFSET) (*(unsigned *)(SPI + OFFSET))
#define ACCESS_GPIO(OFFSET) (*(unsigned *)(GPIO + OFFSET))

void * SPI, * GPIO;

int8_t readRegister(uint8_t reg);
void readAxisData(short * accelData);
void writeRegister(uint8_t reg, uint8_t data);
int getTemp();
short readAxis(uint8_t reg);

int main () {
	size_t length = MAP_SIZE;
	char * UIO_1 = "/dev/uio1";
	// char * GPIO = "/dev/mem";
	int uio_fd;
	if((uio_fd = open(UIO_1, O_RDWR)) < 0) {
		fprintf(stderr, "Failed to open the UIO driver.\n");
	}

	// if((gpio_fd = open(GPIO, O_RDWR)) < 0) {
	// 	fprintf(stderr, "Failed to open /dev/mem for gpio.\n");
	// }
	
	GPIO = mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, uio_fd, 0);
	if(GPIO == -1) {
		int errsv = errno;
		fprintf(stderr, "\tMMAP failed. \n");
		fprintf(stderr, "\tERROR CODE: %s\n", strerror(errsv));
	}

	printf("getpagesize() returns %x\n", getpagesize());
	SPI = mmap(0, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, uio_fd, getpagesize());
	if(SPI == -1) {
		int errsv = errno;
		fprintf(stderr, "\tMMAP failed. \n");
		fprintf(stderr, "\tERROR CODE: %s\n", strerror(errsv));
	}



	// SPI = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd, SPI_OFFSET);
	// if(SPI == -1) {
	// 	int errsv = errno;
	// 	fprintf(stderr, "\tMMAP failed. \n");
	// 	fprintf(stderr, "\tERROR CODE: %s\n", strerror(errsv));
	// }
	
	fprintf(stderr, "UIO is mapped @ 0x%x\n", SPI);
	fprintf(stderr, "GPIO is mapped @ 0x%x\n", SPI);
	/*************************************** 
	 * Set device to Active-low clock
	 * Set device to Manual Slave Select
	 * Set Clock Phase mode to high
	 **************************************/
	ACCESS_REG(SPI_SRR) = 0x0000000a;  //SOFTWARE RESET OF SPI HARDWARE 
	usleep(1000);
	//ACCESS_REG(SPI_CR) = 0x0000007C; // Set controls for the device
	ACCESS_REG(SPI_CR) = 0x000000FC; // Set controls for the device
	ACCESS_REG(SPI_GLB_INT) = 0x00000000; // Disable global interrupt
	ACCESS_REG(SPI_SS) = -1; /* Select Slave*/
	ACCESS_REG(SPI_IIER) = INT_TX_EMPTY;
	printf("SPI bus configured\n");

	/*************************************** 
	 * GPIO Configuration for interrupts
	 * Set to input mode
	 **************************************/
	ACCESS_GPIO(0x4) = 0xFFFFFFFF;

	/*******************************************************
	 * Gyro Device Configuration
	 ******************************************************/
	writeRegister(CTRL_REG4, 0x00);
	writeRegister(CTRL_REG2, 0x29);
	writeRegister(CTRL_REG3, 0x08);
	writeRegister(CTRL_REG5, 0x12); // HPF enabled and data is filtered
	writeRegister(CTRL_REG1, 0x3F); /* Control register 1 */
				/* Set data rate to 100Hz, enable device, enable each axis */
	printf("GYRO configured\n");


	short accelData[3];
	// int8_t status = 0;
	int8_t interrupt = 0;
	int temperature = 0;
	while(1) {
		interrupt = ACCESS_GPIO(0x0);
		if(interrupt & 0x02) {
			usleep(10);
			readAxisData(accelData);
			printf("X-axis data:\t%d\t\tY-axis data:\t%d\t\tZ-axis data:\t%d\t\n", accelData[0], accelData[1], accelData[2]);
		}

		// printf("Temperature is: %d \n", getTemp());
		// sleep(1);

	}
	munmap(SPI, _SC_PAGE_SIZE);

}

void writeRegister(uint8_t reg, uint8_t data) {
	//printf("reg to write%x\tdata %x\n", reg, data);
	uint8_t dummy[2] = {0, 0};
	ACCESS_REG(SPI_SS) = -2; /* Select Slave*/
	ACCESS_REG(SPI_SS) |= RST_TX | RST_RX;
	ACCESS_REG(SPI_DTX) = reg;
	ACCESS_REG(SPI_DTX) = data;

	ACCESS_REG(SPI_CR) |= SPI_ENABLE;

	while(!(ACCESS_REG(SPI_IISR) & INT_TX_EMPTY)) {
		/* Wait til tx fifo is empty */
	}
	ACCESS_REG(SPI_SS) = -1; /* Select Slave*/
	ACCESS_REG(SPI_CR) &= ~SPI_ENABLE;
	ACCESS_REG(SPI_IISR) |= INT_TX_EMPTY; // Toggle the interrupt bit
	
	/* Clear the receive FIFO */
	dummy[0] = ACCESS_REG(SPI_DRX);
	dummy[1] = ACCESS_REG(SPI_DRX);
}

int8_t readRegister(uint8_t reg) {
	//printf("reg to read\t%x\n", reg);
	ACCESS_REG(SPI_SS) = -2; /* Select Slave*/
	int8_t transmit[2] = {0, 0};
	int8_t receive[2] = {0, 0};
	ACCESS_REG(SPI_CR) |= RST_TX | RST_RX;
	transmit[0] = READ | reg;

	ACCESS_REG(SPI_DTX) = transmit[0];
	ACCESS_REG(SPI_DTX) = transmit[1];
	ACCESS_REG(SPI_CR) |= SPI_ENABLE;


	while(!(ACCESS_REG(SPI_IISR) & INT_TX_EMPTY)){
		/* WAit til the tx fifo is empty */
	}
	ACCESS_REG(SPI_SS) = -1; /* Select Slave*/
	ACCESS_REG(SPI_CR) &= ~SPI_ENABLE;
	ACCESS_REG(SPI_IISR) |= INT_TX_EMPTY; // Toggle the interrupt bit
		
	receive[0] = ACCESS_REG(SPI_DRX);
	receive[1] = ACCESS_REG(SPI_DRX);


	//printf("collected data\t%x\n", receive[1]);
	return receive[1];

}

void readAxisData(short * accelData){
	uint8_t upperData = 0, lowerData = 0;

	for (uint8_t i = 0; i < 3; i++) {
		
		accelData[i] = readAxis(OUT_X_L + (i * 2));
	}
	//printf("X-axis data:\t%d\t\tY-axis data:\t%d\t\tZ-axis data:\t%d\t\n", accelData[0], accelData[1], accelData[2]);
	accelData[0] = readAxis(OUT_X_L);

} 

int getTemp() {
	int temperature = readRegister(OUT_TEMP);
	temperature += 14;
	temperature = 32 + (1.8 * temperature);
	return temperature;
}

short readAxis(uint8_t reg) {
	//printf("reg to read\t%x\n", reg);
	ACCESS_REG(SPI_SS) = -2; /* Select Slave*/
	int8_t transmit[3] = {0, 0, 0};
	int8_t receive[3] = {0, 0, 0};
	ACCESS_REG(SPI_CR) |= RST_TX | RST_RX;
	transmit[0] = READ | reg | INCR_REG;

	ACCESS_REG(SPI_DTX) = transmit[0];
	ACCESS_REG(SPI_DTX) = transmit[1];
	ACCESS_REG(SPI_DTX) = transmit[2];
	ACCESS_REG(SPI_CR) |= SPI_ENABLE;


	while(!(ACCESS_REG(SPI_IISR) & INT_TX_EMPTY)){
		/* WAit til the tx fifo is empty */
	}
	ACCESS_REG(SPI_SS) = -1; /* Select Slave*/
	ACCESS_REG(SPI_CR) &= ~SPI_ENABLE;
	ACCESS_REG(SPI_IISR) |= INT_TX_EMPTY; // Toggle the interrupt bit
		
	receive[0] = ACCESS_REG(SPI_DRX);

	receive[1] = ACCESS_REG(SPI_DRX);

	receive[2] = ACCESS_REG(SPI_DRX);

	// printf("R0: %x\tR1: %x\t R2: %x\n", receive[0], receive[1], receive[2]);

	short result = receive[1];
	result |= (receive[2] << 8);
	// printf("received data: %x \n", result);
	return (receive[2] << 8) | receive[1];
}
