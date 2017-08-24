#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

int main(){
	char buf[30] = "/dev/ttyUL4";
	struct termios uart1,old;
	int fd;
	char buf2[20] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't'};
	char receive[20];
	fd = open(buf, O_RDWR | O_NOCTTY);
	if(fd < 0) printf("port failed to open\n");
	//fprintf(stderr, "Error is %s\n", strerror(errno));

	tcgetattr(fd,&old);
	bzero(&uart1,sizeof(uart1));
	
	
	uart1.c_cflag = B9600 | CS8 | CLOCAL | CREAD | CRTSCTS;
	uart1.c_iflag = IGNPAR | ICRNL;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;
	uart1.c_cc[VTIME] = 0;
	uart1.c_cc[VMIN]  = 0;
	
	
	/*	
	cfmakeraw(&uart1);
	cfsetospeed(&uart1, B9600);
	cfsetispeed(&uart1, B9600);
        */
	
	//clean the line and set the attribut
	tcflush(fd,TCIFLUSH);
        tcsetattr(fd,TCSANOW,&uart1);
				 

	write(fd, buf2, 20);
	fprintf(stderr, "Write Complete\n");
	/*read(fd, receive, 16);
	fprintf(stderr, "Read Complete\n");
	char * index = receive;
	for(int i = 0; i < 16; i++) {
		printf("%c ", *index++);
	}*/
	close(fd);
	return 0;
}
