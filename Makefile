CC=gcc -Wall
ARCH= arm

SOURCES= uiotest.c uio-user.c
HEADERS= uio-user.h
EXECUTABLE= uiotest	

SOURCES1=ioxptest.c uio-user.c i2c-fpga-driver.c
HEADERS1= uio-user.h i2c-fpga.h
EXECUTABLE1= ioxptest

$(EXECUTABLE): $(SOURCES) $(HEADERS)        
	$(CC) -std=gnu99 -g -o $(EXECUTABLE) $(SOURCES)

$(EXECUTABLE1): $(SOURCES1) $(HEADERS1)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE1) $(SOURCES1)

.PHONY: clean

clean:  
	rm -f *.o *~ $(EXECUTABLE) $(EXECUTABLE1) 
