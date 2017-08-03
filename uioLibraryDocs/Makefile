CC=gcc -Wall
ARCH= arm

SOURCES= uiotest.c uio-user.c
HEADERS= uio-user.h
EXECUTABLE= uiotest	

SOURCES1=ioxptest.c uio-user.c i2c-fpga-driver.c gpio-fpga-driver.c
HEADERS1= uio-user.h i2c-fpga.h gpio-fpga.h
EXECUTABLE1= ioxptest

SOURCES2= ad2test.c uio-user.c i2c-fpga-driver.c
HEADERS2= uio-user.h i2c-fpga.h
EXECUTABLE2= ad2test

SOURCES3= oledrgbtest.c uio-user.c spi-fpga-driver.c gpio-fpga-driver.c
HEADERS3 = uio-user.h gpio-fpga.h spi-fpga.h
EXECUTABLE3= oledrgb

$(EXECUTABLE): $(SOURCES) $(HEADERS)        
	$(CC) -std=gnu99 -g -o $(EXECUTABLE) $(SOURCES)

$(EXECUTABLE1): $(SOURCES1) $(HEADERS1)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE1) $(SOURCES1)

$(EXECUTABLE2): $(SOURCES2) $(HEADERS2)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE2) $(SOURCES2)

$(EXECUTABLE3): $(SOURCES3) $(HEADERS3)
	$(CC) -std=gnu99 -g -o $(EXECUTABLE3) $(SOURCES3)

.PHONY: clean

clean:  
	rm -f *.o *~ $(EXECUTABLE) $(EXECUTABLE1) 
