
NAVN=kort
MCU=atxmega64a3

# Alle C-filene vi skal bygge:
CFILER=main.c pwm.c com.c dmx.c xmega_usart.c xmega_clock.c ring.c

# Optimisering:
OPTIMISERING=s
# s er size, 0 skrur av optimering 1, 2, 3 er hastighet
# men det er litt lotto hvem av '3' og 's' som er raskest/minst
# DMX-koden kan være sensitiv, så ikke tukle uten å teste

CFLAGG=-I. -g -mmcu=$(MCU) -O$(OPTIMISERING) -fpack-struct -fshort-enums -Wall -Wextra -Wstrict-prototypes 
# Disse flaggene var med, men jeg har fjernet dem:
# -funsigned-bitfields  - Vi bruker ikke bitfields
# -funsigned-char       - Ikke skriv kode hvor bokstavene dine kan være negative
# -Warray-bounds=0      - Dette er et lavere nivå enn det -Wall gir
# Dette tror jeg var en besvergelse for å generere .lst-filer som vi så ikke så på:
# -Wa,-ahlms=$(firstword $(filter %.lst, $(<:.c=.lst)))  

all: hex

stats: $(NAVN).out
	avr-objdump -h $<
	avr-size $<

hex: $(NAVN).hex $(NAVN).ee.hex

AVRDUDE_OPTIONS= -c atmelice_pdi -P usb -p $(MCU) -B 125kHz

# -e betyr gjør chiperase først
writeflash: $(NAVN).hex 
	avrdude $(AVRDUDE_OPTIONS) -e -U flash:w:$<

# -F :Override invalid signature or initialisation check
writeeeprom: $(NAVN).ee.hex
	avrdude $(AVRDUDE_OPTIONS) -F -U eeprom:w:$<

OBJDEPS=$(CFILER:.c=.o)

$(NAVN).out: $(OBJDEPS) 
	avr-gcc -mmcu=$(MCU) -o $@ $^

%.o: %.c 
	avr-gcc $(CFLAGG) -c $< -o $@

%.hex: %.out
	avr-objcopy -j .text -j .data -O ihex $< $@

%.ee.hex: %.out
	-avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@
  
clean:
	rm -f $(NAVN).out $(NAVN).out.map
	rm -f $(OBJDEPS)
	rm -f *.hex
	
.PHONY: all hex writeflash clean stats
