SERIAL_PARAMS := -DUSES_USART1_TX_ISR -DUSES_USART1_RX_ISR
BUILD_TYPE := donglemod
TARGET_DEVICE := CC1110
CC := sdcc

main_module = main.rel

TARGET_DEVICE ?= CC1110

CODE_LOC := 0x000
CODE_LOC_NAME := STDLOC

TARGET_BUILD := ${BUILD_TYPE}

CC ?= sdcc

ifeq ($(CC),sdcc)
	LDFLAGS+=--xram-loc 0xf000 --model-medium --xram-size 0x1000 --code-loc ${CODE_LOC} --code-size 0x8000
  CFLAGS+=--model-medium --verbose
endif

CFLAGS+=-I. -I${SERIAL_TYPE} -D${SERIAL_PARAMS}

default: output output/${TARGET_BUILD} output/${TARGET_BUILD}/${TARGET_BUILD}.hex

common_modules = $(main_module) subg_rfspy.rel radio.rel timer.rel encoding.rel manchester.rel \
	           fourbsixb.rel commands.rel hardware.rel packet_buffer.rel statistics.rel serial.rel

clean:
	rm -rf output/${TARGET_BUILD}

%.rel: %.c
	$(CC) $(CFLAGS) -o output/${TARGET_BUILD}/$@ -c $<

output/${TARGET_BUILD}/${TARGET_BUILD}.hex: $(common_modules) $(extra_modules) $(REL)
	cd output/${TARGET_BUILD} && $(CC) $(LDFLAGS) $(CFLAGS) $(common_modules) $(extra_modules) $(REL) -o ${TARGET_BUILD}.hex

install: output/${TARGET_BUILD} output/${TARGET_BUILD}/${TARGET_BUILD}.hex
	sudo cc-tool -n ${TARGET_DEVICE} --log install.log -ew output/${TARGET_BUILD}/${TARGET_BUILD}.hex

output:
	mkdir output

output/${TARGET_BUILD}: output
	mkdir -p output/${TARGET_BUILD}/

output/${TARGET_BUILD}/%.rel : %.c Makefile
