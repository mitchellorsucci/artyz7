CC=gcc -Wall
ARCH= arm

SOURCES= uiotest.c uio-user.c
HEADERS= uio-user.h
EXECUTABLE= uiotest	

SOURCES1=ioxptest.c uio-user.c i2c-fpga-driver.c
HEADERS1= uio-user.h i2c-fpga.h
EXECUTABLE1= ioxptest

SOURCES2= ad2test.c uio-user.c i2c-fpga-driver.c
HEADERS2= uio-user.h i2c-fpga.h
EXECUTABLE2= ad2test

$(EXECUTABLE): $(SOURCES) $(HEADERS)        
	$(CC) -std=gnu99 -g -o $(EXECUTABLE) $(SOURCES)

$(EXECUTABLE1): $(SOURCES1) $(HEADERS1)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE1) $(SOURCES1)

$(EXECUTABLE2): $(SOURCES2) $(HEADERS2)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE2) $(SOURCES2)
.PHONY: clean

clean:  
	rm -f *.o *~ $(EXECUTABLE) $(EXECUTABLE1) 
