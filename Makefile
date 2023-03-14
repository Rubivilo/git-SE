PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS) -D CPU_MKL46Z256VLL4


CFLAGS=-I./drivers $(COMMONFLAGS)

LDFLAGS=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Tlink.ld

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
RM=rm -f

HELLO=hello_world
BLINK=led_blinky

OBJH=$(patsubst %.c, %.o, $(SRCH))
OBJL=$(patsubst %.c, %.o, $(SRCL))


SRCH=$(filter-out drivers/pin_mux_led.c led_blinky.c, $(wildcard drivers/*.c *.c))
SRCL=$(filter-out drivers/pin_mux_hello.c hello_world.c, $(wildcard drivers/*.c *.c))

clean:
	$(RM) *.elf *.o drivers/*.o

$(HELLO).elf: $(OBJH)
	$(LD) $(LDFLAGS) $(OBJH) -o $@
$(BLINK).elf: $(OBJL)
	$(LD) $(LDFLAGS) $(OBJL) -o $@

flash_hello: $(HELLO).elf
	openocd -f openocd.cfg -c "program $(HELLO).elf verify reset exit"
flash_led:$(BLINK).elf
	openocd -f openocd.cfg -c "program $(BLINK).elf verify reset exit"

