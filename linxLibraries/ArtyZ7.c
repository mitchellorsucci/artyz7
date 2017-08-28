#include "ArtyZ7.h"

static void populateGpioData();

SPIdata * spiDevices[MAX_DEVICE_NUM_COMM];    
I2Cdata * i2cDevices[MAX_DEVICE_NUM_COMM];
UARTdata * uartDevices[MAX_DEVICE_NUM_COMM];
PWM pwmDevice = NULL;
GPIOdata * gpioMeta = NULL;

int ArtyInit() {
    for(int i = 0; i < MAX_DEVICE_NUM_COMM; i++) {
        spiDevices[i] = NULL;
        i2cDevices[i] = NULL;
        uartDevices[i] = NULL;
    }

    int gpioNum = -1;
    int pwmNum = -1;
    for(int i = 0; i < 2; i++) {
        char buf[50];
        char c[4];
        sprintf(buf, "%s%s%d%s", UIO_BASE, "uio", i, "/name");
        FILE * uios;
        uios = fopen(buf, "r");
        if((NULL == uios) && (i == 0)) {
            perror("");
            fprintf(stderr, "There are no PWM or GPIO devices\n");
            pwmDevice = NULL;
            gpioMeta = NULL;
            break;
        } else if (NULL == uios) {
            break; 
        }else {
            fgets(c, 4, uios);
            int test = strcmp(c, "PWM");
            if(test == 0) {
                pwmNum = i;
            } else {
                gpioNum = i;
            }
        }
    }
    if(pwmNum > -1) {
        pwmDevice = PWM_init(pwmNum, 0);
        fprintf(stderr, "PWM Device mapped as uio%d\n", pwmNum); 
    }
    if(gpioNum > -1) {
        gpioMeta = (GPIOdata *) malloc(sizeof(GPIOdata));
        (*gpioMeta).gpioDevice = GPIO_init(gpioNum, 0);
        (*gpioMeta).uioNum = gpioNum;
        populateGpioData();
        fprintf(stderr, "GPIO Device mapped as uio%d\n", gpioNum);
    }
    
    return 0;
}

int ArtyDeInit() {
    if(NULL != pwmDevice) {
        PWM_Close(pwmDevice);
        pwmDevice = NULL;
    }
    if(NULL != gpioMeta) {
        GPIO_Close((*gpioMeta).gpioDevice);
        free(gpioMeta);
        gpioMeta = NULL;
    }

    return 0;
}

static void populateGpioData() {
    char filePath[100];
    char addressString[11];
    /************************************************
    *   Get path information from UIO directory
    ************************************************/
    sprintf(filePath, "%suio%d/maps/map0/addr", UIO_BASE, (*gpioMeta).uioNum);
    FILE * addressPath = fopen(filePath, "r");
    if(NULL == addressPath) {
        fprintf(stderr, "uioNum = %d\n", (*gpioMeta).uioNum);
        fprintf(stderr, "The specified address file for UIO does not exist\n");
        return;
    }
    if(NULL == fgets(addressString, 11, addressPath)) {
        fprintf(stderr, "Unable to read from address UIO file\n");
        return;
    }

    char * adx = &(addressString[2]);
    char dtPath[50];
    sprintf(dtPath, "/proc/device-tree/gpio@%s/", adx);
    char isDualPath[50];
    char chan1Path[50];
    sprintf(isDualPath, "%sxlnx,is-dual", dtPath);
    sprintf(chan1Path, "%sxlnx,gpio-width", dtPath);
    

    /************************************************
    *   Get dual channel information from device-tree
    ************************************************/
    uint8_t dualInfo[5];
    FILE * dualFile = fopen(isDualPath, "r");
    if(NULL == dualFile) {
        fprintf(stderr, "Unable to open Dual channel file\n");
        return;
    }
    if(NULL == fgets(dualInfo, 5, dualFile)) {
        fprintf(stderr, "Unable to read dual Channel info\n");
        return;
    }
    (*gpioMeta).isDual = dualInfo[3];

    
    /************************************************
    *   Get channel 1 information from device-tree
    ************************************************/
    uint8_t chan1Info[5];
    FILE * chan1File = fopen(chan1Path, "r");
    if(NULL == chan1File) {
        fprintf(stderr, "Unable to open channel 1 file\n");
        return;
    }
    if(NULL == fgets(chan1Info, 5, chan1File)) {
        fprintf(stderr, "Unable to read channel 1 info\n");
        return;
    }
    (*gpioMeta).channel1Width = chan1Info[3];

    
    /************************************************
    *   Get channel 2 information from device-tree
    ************************************************/
    if((*gpioMeta).isDual) {
        char chan2Path[50];
        sprintf(chan2Path, "%sxlnx,gpio2-width", dtPath);
        uint8_t chan2Info[5];
        FILE * chan2File = fopen(chan2Path, "r");
        if(NULL == chan1File) {
            fprintf(stderr, "Unable to open channel 2 file\n");
            return;
        }
        if(NULL == fgets(chan2Info, 5, chan2File)) {
            fprintf(stderr, "Unable to read channel 2 info\n");
            return;
        }
        (*gpioMeta).channel2Width = chan2Info[3];
    } else {
        (*gpioMeta).channel2Width = 0;
    }

    /*
    fprintf(stderr, "Is-Dual: %d\n", (*gpioMeta).isDual);
    fprintf(stderr, "CH1-Width: %d\n", (*gpioMeta).channel1Width);
    fprintf(stderr, "CH2-Width: %d\n", (*gpioMeta).channel2Width);
    */
}

int ArtySetPinMode(uint8_t pin, uint8_t mode) {
    if(NULL == gpioMeta || (*gpioMeta).gpioDevice == NULL) {
        fprintf(stderr, "There is no GPIO device\n");
        return 1;
    } else {
        if(pin > ((*gpioMeta).channel1Width - 1)) {
            setPinMode((*gpioMeta).gpioDevice, 2, pin - (*gpioMeta).channel1Width + 1, mode);
        } else {
            setPinMode((*gpioMeta).gpioDevice, 1, pin + 1, mode);
        }
    }

    return 0;
}

int ArtyDigitalWrite(uint8_t pin, uint8_t value) {
    if(NULL == gpioMeta || (*gpioMeta).gpioDevice == NULL) {
        fprintf(stderr, "There is no GPIO device\n");
        return 1;
    } else {
        if (pin > ((*gpioMeta).channel1Width - 1)) {
            digitalWrite((*gpioMeta).gpioDevice, 2, (pin - (*gpioMeta).channel1Width) + 1, value);
        } else {
            digitalWrite((*gpioMeta).gpioDevice, 1, pin + 1, value);
        }
    }

    return 0;
}

int ArtyDigitalRead(uint8_t pin) {
    if(NULL == gpioMeta || (*gpioMeta).gpioDevice == NULL) {
        fprintf(stderr, "There is no GPIO device\n");
        return 1;
    } else {
        if (pin > ((*gpioMeta).channel1Width - 1)) {
            return digitalRead((*gpioMeta).gpioDevice, 2, pin - (*gpioMeta).channel1Width + 1);
        } else {
            return digitalRead((*gpioMeta).gpioDevice, 1, pin + 1);
        }
    }

    return 0;
}

int ArtyGetDIOChannels(uint8_t * numChannels, uint8_t * channelArray) {
    if(NULL == gpioMeta || (*gpioMeta).gpioDevice == NULL) {
        fprintf(stderr, "There is no GPIO device\n");
        return 1;
    }

    *numChannels = (*gpioMeta).channel1Width + (*gpioMeta).channel2Width;
    uint8_t * index = channelArray;
    for(int i = 0; i < *numChannels; i++) {
        *index = i;
        index++;
    }
    return 0;
}

int ArtyPWMenable() {
    if(NULL == pwmDevice) {
        fprintf(stderr, "There is no PWM device to enable\n");
        return 1;
    } else {
        PWM_Enable(pwmDevice);
    }
    
    return 0;
}

int ArtyPWMdisable() {
    if(NULL == pwmDevice) {
        fprintf(stderr, "There is no PWM device to disable\n");
        return 1;
    } else {
        PWM_Disable(pwmDevice);
    }

    return 0;
}

// Sets Frequency for all PWM channels
int ArtyPWMSetFrequency(unsigned long nano) {
    if(NULL == pwmDevice) {
        fprintf(stderr, "There is no PWM device\n");
        return 1;
    } else {
        setPwmPeriod(pwmDevice, nano / 10);
    }

    return 0;
}

int ArtyPWMSetDuty(uint8_t channel, unsigned long nano) {
    if(NULL == pwmDevice) {
        fprintf(stderr, "There is no PWM device\n");
        return 1;
    } else {
        setPwmDuty(pwmDevice, channel + 1, nano / 10);
    }

    return 0;
}


/* Opens a new SPI channel 
    returns 1 if the port has already been opened
    returns 0 on success                               

    Sets SPI channel to SPI_MODE_0
    Sets MaxSpeed to 100Khz
    Sets CS to 0
    Sets MSB mode to MSB
    Sets bit transfer width to 8            */
int ArtySpiOpenMaster(uint8_t channel) {
    if(NULL != spiDevices[channel]) {
        fprintf(stderr, "A Spi device already exists on this channel: %d\n", channel);
        return 1;
    }

    char port[50];
    sprintf(port, "/dev/spidev%d.0", GET_SPI_CHANNEL(channel));

    spiDevices[channel] = (SPIdata *) malloc(sizeof(SPIdata));
    if(((*spiDevices[channel]).spiFD = open(port, O_RDWR)) < 0) {
        perror("Failed to open SPI port: ");
        free(spiDevices[channel]);
        spiDevices[channel] = NULL;
        return 1;
    }

    uint8_t bits = 8;
    int status;
    if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_WR_BITS_PER_WORD, &bits)) < 0) {
        perror("Failed to set packet size on writes: ");
        return 1;
	}
	if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_RD_BITS_PER_WORD, &bits)) < 0) {
        perror("Failed to set packet size on reads: ");
        return 1;
	}

    ArtySpiSetBitOrder(channel, MSBFIRST);  // Default to MSB first
    ArtySpiSetMode(channel, SPI_MODE_0);    // Default to Spi Mode 0
    (*spiDevices[channel]).maxSpeed = ArtyGetSpiSpeed(channel);
    ArtySpiSetMaxSpeed(channel, 0);
    (*spiDevices[channel]).cs = 0;          // Default to CS 0
    return 0;

}

int ArtySpiSetBitOrder(uint8_t channel, uint8_t bitOrder) {
    if(NULL == spiDevices[channel]) {
        fprintf(stderr, "A Spi device has not been opened on this channel: %d\n", channel);
        return 1;
    }
    
    (*spiDevices[channel]).bitOrder = bitOrder;
    return 0;
}

int ArtySpiSetMode(uint8_t channel, uint8_t mode) {
    if(NULL == spiDevices[channel]) {
        fprintf(stderr, "A Spi device has not been opened on this channel: %d\n", channel);
        return 1;
    }

    int status;
    if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_WR_MODE, &mode)) < 0) {
        perror("Failed to Set SPI Write Mode: ");
        return 1;
	}
	if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_RD_MODE, &mode)) < 0) {
        perror("Failed to Set SPI Read Mode: ");
        return 1;
	}
    (*spiDevices[channel]).mode = mode;

    return 0;

}


int ArtySpiSetMaxSpeed(uint8_t channel, unsigned long speed) {
    if(NULL == spiDevices[channel]) {
        fprintf(stderr, "A Spi device has not been opened on this channel: %d\n", channel);
        return 1;
    }

    int status;
    if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_WR_MAX_SPEED_HZ, &(*spiDevices[channel]).maxSpeed)) < 0) {
        perror("Failed to Set SPI Write Speed: ");
        return 1;
	}
	if((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_RD_MAX_SPEED_HZ, &(*spiDevices[channel]).maxSpeed)) < 0) {
        perror("Failed to Set SPI Read Speed: ");
        return 1;
	}

    return 0;
}


int ArtySpiTransfer(uint8_t channel, uint8_t * tx_buffer, uint8_t * rx_buffer, uint8_t numBytes) {
    if(NULL == spiDevices[channel]) {
        fprintf(stderr, "A Spi device has not been opened on this channel: %d\n", channel);
        return 1;
    }

    if((*spiDevices[channel]).bitOrder == LSBFIRST) {
        for(int i = 0; i < numBytes; i++) {
            *(tx_buffer + i) = reverseBits(*(tx_buffer + i));
        }
    }

    struct spi_ioc_transfer transfer = {
        .tx_buf = tx_buffer,
        .rx_buf = rx_buffer,
        .len = numBytes,
        .delay_usecs = 0,
        .speed_hz = 0,
        .bits_per_word = 0,
    };

    int status;
    if ((status = ioctl((*spiDevices[channel]).spiFD, SPI_IOC_MESSAGE(1), &transfer)) < 0) {
        perror("FAILED TO SEND SPI DATA: ");
        return -1;
    }
    
    return status;
}

/* Returns NULL if the file does not exist */
/* Returns the spi transfer speed set in the FPGA hardware */
unsigned int ArtyGetSpiSpeed(uint8_t channel) {
    unsigned int result = 0;
    char filePath[100];
    sprintf(filePath, "/sys/class/spi_master/spi%d/of_node/spidev@0/spi-max-frequency", GET_SPI_CHANNEL(channel));
    FILE * maxFreq = fopen(filePath, "r");
    if(NULL == maxFreq) {
        fprintf(stderr, "The spi-max-frequency file does not exist\n");
        return 1;
    }
    unsigned char fileData[5]; // 32 bit value to be read....4 locations plus terminating character
    fgets(fileData, 5, maxFreq);
    unsigned char * index = fileData;
    for(int i = 3; i > -1; i--) {
        unsigned int tempValue = *index; // Get the data stored and convert to 32 bit value
        for(int j = 1; j <= i; j++) {
            tempValue *= 256; // Do the hexadecimal to decimal conversion
        }
        result += tempValue;
        index++;
    }
    return result;
}

int ArtySpiCloseMaster(uint8_t channel) {
    if(NULL == spiDevices[channel]) {
        fprintf(stderr, "A Spi device does not exist on this channel: %d\n", channel);
        return 1;
    }

    close((*spiDevices[channel]).spiFD);
    free(spiDevices[channel]);
    spiDevices[channel] = NULL;
    return 1;
}

int ArtyI2COpenMaster(uint8_t channel) {
    if(NULL != i2cDevices[channel]) {
        fprintf(stderr, "An I2C device already exists on this channel: %d\n", channel);
        return 1;
    }

    char port[50];
    sprintf(port, "%s%d", I2C_BASE_PORT, channel);
    i2cDevices[channel] = (I2Cdata *)malloc(sizeof(I2Cdata));

    if(((*i2cDevices[channel]).i2cFD = open(port, O_RDWR)) < 0) {
        perror("Failed to open I2C port: ");
        free(i2cDevices[channel]);
        i2cDevices[channel] = NULL;
        return 1;
    }

    return 0;
}

int ArtyI2CWrite(uint8_t channel, uint8_t slaveAdx, uint8_t * tx_buffer, uint8_t numBytes) {
    if(NULL == i2cDevices[channel]) {
        fprintf(stderr, "An I2C device does not exist on this channel: %d\n", channel);
        return 1;
    }

    if(ioctl((*i2cDevices[channel]).i2cFD, I2C_SLAVE, slaveAdx) < 0) {
        perror("Failed to register slave on I2C bus: ");
        return 1;
    }

    if(write((*i2cDevices[channel]).i2cFD, tx_buffer, numBytes) != numBytes) {
        fprintf(stderr, "Failed to write all I2C data\n");
        return 1;
    }

    return 0;
}

int ArtyI2CRead(uint8_t channel, uint8_t slaveAdx, uint8_t * rx_buffer, uint8_t numBytes) {
    if(NULL == i2cDevices[channel]) {
        fprintf(stderr, "An I2C device does not exist on this channel: %d\n", channel);
        return 1;
    }

    if(ioctl((*i2cDevices[channel]).i2cFD, I2C_SLAVE, slaveAdx) < 0) {
        perror("Failed to register slave on I2C bus: ");
        return 1;
    }

    if(read((*i2cDevices[channel]).i2cFD, rx_buffer, numBytes) != numBytes) {
        fprintf(stderr, "Failed to read all I2C data\n");
        return 1;
    }

    return 0;
}

int ArtyI2CCloseMaster(uint8_t channel) {
    if(NULL == i2cDevices[channel]) {
        fprintf(stderr, "An I2C device does not exist on this channel: %d\n", channel);
        return 1;
    }

    close((*i2cDevices[channel]).i2cFD);
    free(i2cDevices[channel]);
    i2cDevices[channel] = NULL;
    return 0;
}


/* The uart port will be opened with whatever baud rate is the default for that
    port on boot                 */
int ArtyUartOpen(uint8_t channel) {
    if(NULL != uartDevices[channel]) {
        fprintf(stderr, "A UART device already exists on this channel: %d\n", channel);
        return 1;
    }

    char port[50];
    sprintf(port, "/dev/ttyS%d", channel);
    uartDevices[channel] = (UARTdata *)malloc(sizeof(UARTdata));
    if(((*uartDevices[channel]).uartFD = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY)) < 0) {
        perror("Failed to open uart port: ");
        free(uartDevices[channel]);
        uartDevices[channel] = NULL;
        return 1;
    }

    return 0;

}

int ArtyUartSetBaudRate(uint8_t channel, unsigned long baud) {
    if(NULL == uartDevices[channel]) {
        fprintf(stderr, "A UART device does not exist on this channel: %d\n", channel);
        return 1;
    }
    uint8_t flag = 0;
    speed_t formattedBaud;
    speed_t baudArray[] = {B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800,
                        B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400};
    unsigned long speeds[] = {50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
                        2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
    
    for(int i = 0; i < (sizeof(speeds) / 4); i++) {
        if(baud == speeds[i]) {
            flag = 1;
            formattedBaud = baudArray[i];
        }
    }
    if(!flag) {
        fprintf(stderr, "The baud rate requested is not supported.\n");
        fprintf(stderr, "The supported baud rates are 50, 75, 110, 134, 150, "
                        "200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, "
                        "38400, 57600, 115200, 230400\n");
        return 1;
    }

    struct termios tty;
    tcgetattr((*uartDevices[channel]).uartFD, &tty);
    int status;
    
    status = cfsetospeed(&tty, formattedBaud);
    if (status < 0) {
        perror("Failed to set output baud rate: ");
        return 1;
    }
    
    status = cfsetispeed(&tty, formattedBaud);
    if (status < 0) {
        perror("Failed to set input baud rate: ");
        return 1;
    }
    
    tcflush((*uartDevices[channel]).uartFD, TCIOFLUSH);
    status = tcsetattr((*uartDevices[channel]).uartFD, TCSANOW, &tty);
    if(status < 0) {
        perror("Failed to set the tty port: ");
        return 1;
    }

    return 0;
}

int ArtyUartGetBytesAvailable(uint8_t channel, uint8_t * numBytes) {
    if(NULL == uartDevices[channel]) {
        fprintf(stderr, "A UART device does not exist on this channel: %d\n", channel);
        return 1;
    }

    int bytes = -1;
    ioctl((*uartDevices[channel]).uartFD, FIONREAD, &bytes);
    if(bytes < 0) {
        return 1;
    } else {
        *numBytes = (uint8_t) bytes;
    }
    
    return 0;
}

int ArtyUartRead(uint8_t channel, uint8_t numBytes, uint8_t * rxbuffer, uint8_t * numBytesRead) {
    if(NULL == uartDevices[channel]) {
        fprintf(stderr, "A UART device does not exist on this channel: %d\n", channel);
        return 1;
    }
    uint8_t bytesAvailable = -1;
    ArtyUartGetBytesAvailable(channel, &bytesAvailable);
    if(bytesAvailable >= numBytes) {
        int bytesRead = read((*uartDevices[channel]).uartFD, rxbuffer, numBytes);
        *numBytesRead = (uint8_t) bytesRead;
        if(bytesRead != numBytes) {
            return 1;
        }
    } else {
        fprintf(stderr, "Not enough bytes available for UART read: ");
        return 1;
    }

    return 0;
}

int ArtyUartWrite(uint8_t channel, uint8_t numBytes, uint8_t * txbuffer) {
    int transmitted = write((*uartDevices[channel]).uartFD, txbuffer, numBytes);
    if(transmitted != numBytes) {
        return 1;
    }
    
    return 0;
}

int ArtyUartClose(uint8_t channel) {
    if(NULL == uartDevices[channel]) {
        fprintf(stderr, "A UART device does not exist on this channel: %d\n", channel);
        return 1;
    }
    
    close((*uartDevices[channel]).uartFD);
    free(uartDevices[channel]);
    uartDevices[channel] = NULL;
    return 0;

}

uint8_t reverseBits(uint8_t b){
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

