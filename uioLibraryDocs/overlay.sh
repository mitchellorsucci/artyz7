#!/bin/bash


DTBO="$2"
BIN="$3"
echo
echo
echo -e "\tOverlay File ------> $DTBO"
echo -e "\tFPGA Bit file ------> $BIN"
echo


start() {
	echo -e "\t**********************************************"
	echo -e "\t          Loading Overlay $DTBO               "
	echo -e "\t**********************************************"
	mkdir /config
	mount -t configfs configfs /config
	mkdir /config/device-tree/overlays/$DTBO
	echo $DTBO > /config/device-tree/overlays/$DTBO/path
	cat /lib/firmware/$BIN > /dev/xdevcfg
	echo
	echo -e "\tOverlay Loaded"
	
}

stop() {
	echo -e "\t**********************************************"
	echo -e "\t          Unloading Overlay $DTBO             "
	echo -e "\t**********************************************"
	rmdir /config/device-tree/overlays/$DTBO
	echo
	echo -e "\tOverlay unloaded"
	
}

case "$1" in
	start)
		start
		;;
	stop)
		stop
		;;
	*)	
		echo -e "\tUsage: $0 (start | stop) ---- 0verlay file ---- bitstream file"
		echo -e "\tsuch as $0 start some_overlay.dtbo some_bitstream.bin"
		echo -e "\tThe .dtbo and .bin files should both be in /lib/firmware"
		exit 1
esac

exit $?

