# Name: Makefile
# Project: VUsbTinyBoot, for Trinket Pro
# Author: Frank Zhao
# Creation Date: 2013-06-06
# Tabsize: 4
# Copyright: (c) 2013 by Adafruit Industries
# License: GNU GPL v2 (see License.txt)

# This makefile is derived from the USBaspLoader makefile
# works best with AVR GCC 4.2.2 (WinAVR 20071221)
###############################################################################
# Configure the following variables according to your AVR.
# Program the device with
#     make fuse    # to set the clock generator, boot section size etc.
#     make flash   # to load the boot loader into flash
#     make lock    # to protect the boot loader from overwriting

F_CPU = 16000000
DEVICE = atmega328p
# if the code size is over 2K (under 4K), the BOOTLOADER_ADDRESS is 1000 for 8K devices, 3000 for 16K and 7000 for 32K
# if the code size is under 2K, the BOOTLOADER_ADDRESS is 1800 for 8K devices, 3800 for 16K and 7800 for 32K
# ATmega8, ATmega88, ATmega168 do not support 4K bootloaders
BOOTLOADER_ADDRESS = 7000
FUSEOPT = $(FUSEOPT_328)
LOCKOPT = -U lock:w:0x2F:m
UNLOCKOPT = -U lock:w:0x3F:m

PROGRAMMER = -c usbtiny
# PROGRAMMER contains AVRDUDE options to address your programmer

FUSEOPT_8 = -U hfuse:w:0xD0:m -U lfuse:w:0xBF:m
FUSEOPT_88 = -U hfuse:w:0xD5:m -U lfuse:w:0xFF:m -U efuse:w:0xF8:m
FUSEOPT_168 = -U hfuse:w:0xD5:m -U lfuse:w:0xFF:m -U efuse:w:0xF8:m
FUSEOPT_328 = -U lfuse:w:0xFF:m -U hfuse:w:0xD0:m -U efuse:w:0x05:m
# You may have to change the order of these -U commands.

###############################################################################

# Tools:
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
CC = avr-gcc


# Arduino classic LED pin on ULPNode Arduino D5
#SET LED=LED=D5

#REM WS2812B Data In avr pin Arduino D6
#SET WS2812=WS2812=D6

#REM Number of flash when entering bootloader
#SET LED_START_FLASHES=LED_START_FLASHES=3

#REM Wake up switch avr pin Arduino D4
#SET WAKE_SWITCH=WAKE_SWITCH=D4

#REM power sensor avr pin Arduino D9
#SET PWR_SENSOR=PWR_SENSOR=B1

#REM power RF Radio module avr pin Arduino D7
#SET PWR_RF=PWR_RF=D7

#REM booster enable avr pin Arduino A2
#SET PWR_BOOST=PWR_BOOST=C2


# Options:

#defines for ULPNode
DEFINES = -DULPNODE=1 -DLED=D5 -DWS2812=D6 -DPWR_SENSOR=B1 -DPWR_RF=D7 -DPWR_BOOST=C2 -DBAUD_RATE=250000

# Remove the -fno-* options when you use gcc 3, it does not understand them ( -fno-move-loop-invariants -fno-tree-scev-cprop -fno-inline-small-functions )
CFLAGS = -Wall -Os -I. -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) -DBOOTLOADER_ADDRESS=0x$(BOOTLOADER_ADDRESS) $(DEFINES)  
#LDFLAGS = -Wl,--relax,--gc-sections -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) 
LDFLAGS = -Wl,--relax -Wl,--section-start=.text=$(BOOTLOADER_ADDRESS) -Wl,--section-start=.functable=0x7ff0  
OBJECTS = usbdrv/usbdrvasm.o main.o optiboot.o

# symbolic targets:
all: main.hex disasm

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

.S.o:
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(CC) $(CFLAGS) -S $< -o $@

flash:	all
	$(AVRDUDE) -B 1 -U flash:w:main.hex:i

readflash:
	$(AVRDUDE) -B 1 -U flash:r:read.hex:i

fuse:
	$(AVRDUDE) $(FUSEOPT)

unlock:
	$(AVRDUDE) $(UNLOCKOPT)

lock:
	$(AVRDUDE) $(LOCKOPT)

read_fuses:
	$(UISP) --rd_fuses

clean:
	rm -f main.hex main.bin *.lst $(OBJECTS)

# file targets:
main.bin:	$(OBJECTS)
	$(CC) $(CFLAGS) -o main.bin $(OBJECTS) $(LDFLAGS)

main.hex:	main.bin
	rm -f main.hex main.eep.hex
#	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avr-objcopy -j .text -j .data -j .functable --set-section-flags .functable=alloc,load -O ihex main.bin main.hex

	avr-size main.hex
	cp main.hex /volume1/partage/devt/ulpnode_bl_${F_CPU}Hz.hex

disasm:	main.bin
	avr-objdump -h -S -D -t main.bin >main.lst

cpp:
	$(CC) $(CFLAGS) -E main.c

# Special rules for generating hex files for various devices and clock speeds
ALLHEXFILES = hexfiles/mega8_12mhz.hex hexfiles/mega8_15mhz.hex hexfiles/mega8_16mhz.hex \
	hexfiles/mega88_12mhz.hex hexfiles/mega88_15mhz.hex hexfiles/mega88_16mhz.hex hexfiles/mega88_20mhz.hex\
	hexfiles/mega168_12mhz.hex hexfiles/mega168_15mhz.hex hexfiles/mega168_16mhz.hex hexfiles/mega168_20mhz.hex\
	hexfiles/mega328p_12mhz.hex hexfiles/mega328p_15mhz.hex hexfiles/mega328p_16mhz.hex hexfiles/mega328p_20mhz.hex

allhexfiles: $(ALLHEXFILES)
	$(MAKE) clean
	avr-size hexfiles/*.hex

$(ALLHEXFILES):
	@[ -d hexfiles ] || mkdir hexfiles
	@device=`echo $@ | sed -e 's|.*/mega||g' -e 's|_.*||g'`; \
	clock=`echo $@ | sed -e 's|.*_||g' -e 's|mhz.*||g'`; \
	addr=`echo $$device | sed -e 's/\([0-9]\)8/\1/g' | awk '{printf("%x", ($$1 - 2) * 1024)}'`; \
	echo "### Make with F_CPU=$${clock}000000 DEVICE=atmega$$device BOOTLOADER_ADDRESS=$$addr"; \
	$(MAKE) clean; \
	$(MAKE) main.hex F_CPU=$${clock}000000 DEVICE=atmega$$device BOOTLOADER_ADDRESS=$$addr DEFINES=-DUSE_AUTOCONFIG=1
	mv main.hex $@



  
  
