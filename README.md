This represents a repository for work that is done on a Digilent ArtyZ7
which integrates a Xilinx Zynq chip.

This chip contains both an ARM processor and FPGA fabric.

The software included in this repository deals with controlling hardware
instantiated in the FPGA fabric from user space in Linux on the ARM chip.

The supported hardware modules include GPIO, PWM, UART, SPI, and I2C.

Additionally, there are multiple ways to control SPI and I2C harwdare.
This repository provides libraries to control this hardware using 
provided linux device drivers from Xilinx as well as custom drivers
that utilize the UIO generic library.

PWM and GPIO are always controlled by UIO drivers.
