
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

# Change this if you use, for example, an ATTiny44.
CHIP = attiny84

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS = -Os -g -mmcu=$(CHIP) -std=c99 $(OPTS) -ffreestanding -Wall

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	blinky.hex

clean:
	rm -f blinky.hex blinky.elf blinky.o

flash:	blinky.hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:blinky.hex

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xdf:m -U lfuse:w:0x62:m -U efuse:w:0xff:m

init:	fuse flash
