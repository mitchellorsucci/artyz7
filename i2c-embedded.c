#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>
#include <stdint.h> //for uint8_t

#define GPO_DATA_OUT_A	0x2A
#define GPO_DATA_OUT_B	0x2B
#define GPO_OUT_MODE_A	0x2D
#define GPO_OUT_MODE_B	0x2E
#define GPIO_DIRECTION_A	0x30
#define GPIO_DIRECTION_B	0x31

#define PWM_OFFT_LOW	0x3E
#define PWM_OFFT_HIGH	0x3F
#define PWM_ONT_LOW		0x40
#define PWM_ONT_HIGH	0x41
#define PWM_CNTRL		0x42

uint8_t readRegister(int fd, char reg);
void writeRegister(int fd, char reg, char data); 

int i2c_adx	=	0x34;
char i2cport[] = "/dev/i2c-1";


void main() {
	int fd;

	if((fd = open(i2cport, O_RDWR)) < 0) {
		printf("FAILED TO OPEN I2C PORT FOR IOXP\n");
		exit(1);
	}

	if(ioctl(fd, I2C_SLAVE, i2c_adx) < 0) {
		printf("FAILED TO LINK WITH IOXP\n");
		exit(1);
	}

	unsigned char setup_1[3] = {GPO_OUT_MODE_A, 0xFF, 0xFF};
	unsigned char setup_2[3] = {GPIO_DIRECTION_A, 0xFF, 0xFF};
	write(fd, setup_1, 3);
	write(fd, setup_2, 3);
	// writeRegister(fd, GPO_OUT_MODE_B, 0xFF);	// Set to open-drain
	// writeRegister(fd, GPIO_DIRECTION_B, 0xFF); // Set the GPIOS to outs
	// writeRegister(fd, GPO_OUT_MODE_A, 0xFF);
	// writeRegister(fd, GPIO_DIRECTION_A, 0xFF);
	writeRegister(fd, GPO_DATA_OUT_A, 0x00);
	writeRegister(fd, GPO_DATA_OUT_B, 0x00);

	while(1) {
		//writeRegister(fd, GPO_DATA_OUT_A, 0xF0);
		writeRegister(fd, GPO_DATA_OUT_B, 0x80);
		usleep(500000);
		//writeRegister(fd, GPO_DATA_OUT_A, 0x00);
		writeRegister(fd, GPO_DATA_OUT_B, 0x00);
		usleep(500000);
	}
}

void writeRegister(int fd, char reg, char data) {
	char packet[2] = {reg, data};
	write(fd, packet, 2);
}

uint8_t readRegister(int fd, char reg) {
	uint8_t buf;
	write(fd, &reg, 1);
	read(fd, &buf, 1);
	return buf;
}