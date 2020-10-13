CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
AS = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy

CFLAGS += -std=c99 -mcpu=cortex-m0 -mthumb -Wall -Wextra -Wpedantic
LDFLAGS = -Tstm32.ld -mcpu=cortex-m0 -mthumb -Wl,--gc-sections -lc -lnosys

CFLAGS += -DSTM32F042x6

CFLAGS += -I/opt/STM32/STM32F0
CFLAGS += -I/usr/arm-none-eabi/include/

ifeq ($(DEBUG),1)
CFLAGS += -Og -ggdb -DDEBUG
else
CFLAGS += -Os
endif

stm32.bin: stm32
	$(OBJCOPY) $^ $@ -O binary

stm32: vec.o main.o boot.o
	$(CC) $^ -o $@ $(LDFLAGS)

flash: stm32.bin
ifeq ($(DEBUG),1)
	openocd -f flash.cfg
else
	st-flash --reset write $^ 0x08000000
endif

clean:
	rm -f *.o stm32 stm32.bin
