#!/bin/sh

OCD_PATH=/home/dimka/STM32Toolchain/openocd
OCD_BIN=${OCD_PATH}/bin/openocd
OUT=/home/dimka/STM32Toolchain/cubemx-out/Lora-Gps-Sender/build/Lora-Gps-Sender
BIN=${OUT}.bin
ELF=${OUT}.elf

${OCD_BIN} -f ${OCD_PATH}/scripts/board/st_f103c8t6.cfg \
	-c "init" \
	-c "reset init" \
	-c "reset halt" \
	-c "flash write_image erase $ELF" \
	-c "verify_image $ELF" \
	-c "reset run" \
	-c "targets" \
	-c "shutdown"
