# Makefile for 2ndloader, Matt'sSerialMultibootCable.

AS = arm-thumb-elf-as
LD = arm-thumb-elf-ld
OBJCOPY = arm-thumb-elf-objcopy
SOURCES = loader.s
OBJS = loader.o


loader.h: loader.gba
	mkinc.pl < loader.gba > loader.h

loader.gba:	loader.elf
	$(OBJCOPY) -O binary loader.elf loader.gba

loader.elf:	$(OBJS)
	$(LD) -Ttext 0 -o loader.elf $(OBJS)

loader.o:	loader.s
	$(AS) -o loader.o loader.s

