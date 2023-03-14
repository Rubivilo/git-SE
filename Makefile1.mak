PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS) -D CPU_MKL46Z26VLL4


CFLAGS=-I./drivers $(COMMONFLAGS)

LDFLAGSH=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Tlink.ld
LDFLAGSL=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Tlink.ld

HELLO=hello_world
BLINK=led_blinky

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

clean:
	$(RM) $(TARGET).srec $(TARGET).elf $(TARGET).bin $(TARGET).map $(OBJ)

$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) $(LDLIBS) -o $@


