#
# Makefile for primo4
#

BUILD_BASE_DIR := ${shell cd ..; pwd}
CROSS_COMPILE := arm-none-eabi-
CC := ${CROSS_COMPILE}gcc
CXX := ${CROSS_COMPILE}g++
LD := ${CROSS_COMPILE}ld
OBJCOPY := ${CROSS_COMPILE}objcopy
SIZE := ${CROSS_COMPILE}size
RM := rm -rf

VPATH := ${shell find .. -path ../test -prune -o \( -name '*.s' -o -name '*.c' -o -name '*.cc' \) -print | sed -e 's:/[^\/]*$$::' | sort -u}
TARGET := primo4.elf

#
# compiler options
#
INCLUDE_FILES := \
  -I${BUILD_BASE_DIR}/Core/Inc \
  -I${BUILD_BASE_DIR}/App/Inc \
  -I${BUILD_BASE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc \
  -I${BUILD_BASE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  -I${BUILD_BASE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
  -I${BUILD_BASE_DIR}/Drivers/CMSIS/Include \
  -I${BUILD_BASE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -I${BUILD_BASE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

DEFINES := \
  -DCORE_CM7 \
  -DUSE_HAL_DRIVER \
  -DSTM32H747xx

BASE_OPTIONS := \
  -mcpu=cortex-m7 \
	--specs=nano.specs \
	-mfpu=fpv5-d16 \
	-mfloat-abi=hard \
	-mthumb

OPTIONS := \
  ${BASE_OPTIONS} \
  -Os \
  -ffunction-sections \
  -fdata-sections \
  -Wall \
  -fstack-usage

ASM_FLAGS := \
  ${BASE_OPTIONS} \
  -x assembler-with-cpp

C_FLAGS := \
  -std=gnu11 \
  ${OPTIONS} \
  ${DEFINES} \
  ${INCLUDE_FILES}

CC_FLAGS := \
  -std=gnu++14 \
  -fno-exceptions \
  -fno-rtti \
  -fno-use-cxa-atexit \
  ${OPTIONS} \
  ${DEFINES} \
  ${INCLUDE_FILES}

DEPFLAGS = -MT $@ -MMD -MP -MF $*.d

SRCS := ${shell find .. -path ../test -prune -o \( -name '*.s' -o -name '*.c' -o -name '*.cc' \) -print | sed -e 's:^.*/::g'}
OBJS := ${shell for i in ${SRCS}; do echo $$i | sed -e 's:\.s$$:.o:' -e 's:\.c$$:.o:' -e 's:\.cc$$:.o:' -e 's:\.cpp$$:.o:'; done}
DEPS := ${OBJS:%.o=%.d}

#
# build
#

all: ${TARGET}

${TARGET}: ${OBJS}
	${CXX} -o $@ ${OBJS} -mcpu=cortex-m7 -T"${BUILD_BASE_DIR}/STM32H747IITX_FLASH.ld" --specs=nosys.specs -Wl,-Map=${TARGET:%.elf=%.map} -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo
	${SIZE} $@
	@echo 'Finished building target: $@'
	@echo

install: all
	${OBJCOPY} -O binary ${TARGET} ${TARGET:%.elf=%.bin}
	st-flash --reset write ${TARGET:%.elf=%.bin} 0x8000000

clean:
	rm -rf *.o *.d *.su *.map *.elf *.bin

include ${wildcard ${DEPS}}

${DEPS}:

%.o: %.s %.d
	echo $@
	echo $^
	${CC} ${ASM_FLAGS} ${DEPFLAGS} -c -o $@ $<

%.o: %.c %.d
	${CC} ${C_FLAGS} ${DEPFLAGS} -c -o $@ $<

%.o: %.cc %.d
	${CXX} ${CC_FLAGS} ${DEPFLAGS} -c -o $@ $<

