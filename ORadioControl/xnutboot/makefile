###############################################################################
# Makefile for the project $(PROJECT)
###############################################################################

## General Flags
PROJECT = xnutboot
MCU = atmega48
CC = 

CPP = avr-gcc

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -Os -mcall-prologues -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -ffunction-sections -MD -MP -MT $(*F).o -MF dep/$(@F).d 


## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,--section-start=.text=0x0b00 
LDFLAGS +=  -Wl,--gc-sections
LDFLAGS +=  -Wl,--Map=$(PROJECT).map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

OBJDUMP=avr-objdump
OBJCOPY=avr-objcopy
SIZE=avr-size

## Build
all: bootloader $(PROJECT).stats $(PROJECT).lss	## Compile

bootloader: $(PROJECT).hex
	
##Compile
%.o: %.c
	$(CPP) $(INCLUDES) $(CFLAGS) -c  $<

##Link
%.elf: %.o $(OBJECTS)
	$(CPP) $(LDFLAGS) $^ $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: %.elf
	$(OBJCOPY) $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

%.stats: %.elf
	$(SIZE) $< > $@
	$(OBJDUMP) -h $< >> $@

## Clean target
.PHONY: clean
clean:
	-rm -rf *.o *.elf *.stats dep/* *.hex *.eep *.lss *.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

