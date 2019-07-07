CROSS_COMPILE = arm-none-eabi-
CC      = $(CROSS_COMPILE)gcc
LD      = $(CROSS_COMPILE)ld -v
CP      = $(CROSS_COMPILE)objcopy
OD      = $(CROSS_COMPILE)objdump
  
CFLAGS  =  -I./ -c -fno-common -O0 -mcpu=cortex-m3 -mthumb
LFLAGS  = -Tstm32f100.ld
CPFLAGS = -Obinary
ODFLAGS = -S

all: test

clean:
	-rm -f *.lst *.o *.elf *.bin

test: dma-demo-1.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) dma-demo-1.elf dma-demo-1.bin
	$(OD) $(ODFLAGS) dma-demo-1.elf > dma-demo-1.lst

dma-demo-1.elf: dma-demo-1.o stm32f100.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o dma-demo-1.elf dma-demo-1.o startup_stm32f10x.o

dma-demo-1.o: dma-demo-1.c
	@ echo ".compiling"
	$(CC) $(CFLAGS) dma-demo-1.c
	$(CC) $(CFLAGS) startup_stm32f10x.c



