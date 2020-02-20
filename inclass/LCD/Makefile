SHELL               = /bin/bash
PRG                 = stim_timer_skel
OBJS                = stim_timer_skel.o hd44780.o 
SRCS                = stim_timer_skel.c hd44780.c 
MCU_TARGET          = atmega128
F_CPU               = 16000000UL
PROGRAMMER_TARGET   = m128

#agressive optimization
OPTIMIZE       = -O2    # options are 1, 2, 3, s
#optimize for small code
#OPTIMIZE       = -Os    # options are 1, 2, 3, s

DEFS                =
LIBS                =
CC                  = avr-gcc

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -DF_CPU=$(F_CPU)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

#prevent confusion with any file named "clean"
#the dash "-" prevents rm from erroring out with file not found
.PHONY	: clean
clean: 
	-rm -rf $(PRG).o $(PRG).elf $(PRG).lst $(PRG).map $(PRG).srec $(PRG)*.bin
	-rm -rf $(PRG)_eeprom.srec $(PRG)_eeprom*.bin $(PRG)_eeprom.hex $(PRG).hex 
	-rm -rf *.d  *.o

#clean entire directory
all_clean:
	rm -rf *.o *.elf *.lst *.map *.srec *.bin *.hex *.d

#Here is the pattern rule to generate a file of dependencies (i.e., a makefile) 
#called `name.d' from a C source file +called `name.c':
%.d: %.c
	@set -e; rm -f $@; \
	$(CC) -MM $(CFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

#include the dependencies from the other makefiles
-include $(SRCS:.c=.d)

#setup for usb programmer
program: $(PRG).hex
#	avrdude -p $(PROGRAMMER_TARGET) -c usbasp -e -U flash:w:$(PRG).hex 
	avrdude -p $(PROGRAMMER_TARGET) -c usbasp -e -U flash:w:$(PRG).hex \
                                                     -U eeprom:w:$(PRG)_eeprom.hex
#	avrdude -p $(PROGRAMMER_TARGET) -c osuisp2 -e -U flash:w:$(PRG).hex 

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }
