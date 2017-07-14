#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <fcntl.h>

#define MAP_SIZE 0x1000
#define GPIO_BASE 0x40000000
/*#define GPIO_DIR (GPIO + 4)*/

int main () {
	char * UIO_0 = "/dev/uio0";
	int uio;
	volatile unsigned * GPIO;

	if((uio = open(UIO_0, O_RDWR)) < 0) {
		fprintf(stderr, "Failed to open the UIO driver\n\r");
		exit(2);
	}
	
	GPIO = (volatile unsigned *) mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, uio, 0);
	if (!GPIO) {
		fprintf(stderr, "Failed to map the UIO driver into virtual memory\n\r");
		exit(2);
	}
	fprintf(stderr, "GPIO is mapped at 0x%x\n", GPIO);
	*(GPIO + 1) = 0x00000000; // Set to GPIOs to output

	while(1){
		for(int i = 0; i < 11; i++) {
			if(i < 1)
				*GPIO = 1;
			else if (i < 2)
				*GPIO = 3;
			else
				*GPIO = 7 << (i - 2);
			usleep(100000);
		}
	}
	
}
