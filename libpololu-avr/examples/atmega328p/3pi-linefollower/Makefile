DEVICE = atmega328p
AVRDUDE_DEVICE = m328p
DEVICE ?= atmega168
AVRDUDE_DEVICE ?= m168

<<<<<<< HEAD
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(DEVICE) -Os -I../../../
CPP=avr-g++
CC=avr-gcc
OBJ2HEX=avr-objcopy 
LDFLAGS=-L../../../ -Wl,-gc-sections -lpololu_$(DEVICE) -Wl,-relax
=======
CFLAGS=-g -I../../.. -Wall -mcall-prologues -mmcu=$(DEVICE) -Os
CPP=avr-g++
CC=avr-gcc
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -L../../.. -lpololu_$(DEVICE) -Wl,-relax
>>>>>>> ed1b5ceafcf44959db9a4184f4b9026d3ef9843e

PORT ?= /dev/ttyUSB0
AVRDUDE=avrdude
TARGET=test

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.obj: %.o
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U flash:w:$(TARGET).hex
