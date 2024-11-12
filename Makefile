#
#

all:
	make -C ./CM7/Release all

install: all
	arm-none-eabi-objcopy -O binary ./CM7/Release/primo4_CM7.elf ./CM7/Release/primo4_CM7.bin
	st-flash --reset write CM7/Release/primo4_CM7.bin 0x8000000

clean:
	make -C ./CM7/Release clean
