CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS += -Wall -O2 -g
CFLAGS += -Iinclude -Iinclude/cmsis -Isrc/common

LDFLAGS = -Tstm32f767.ld -nostartfiles

COMMON_SRC = $(wildcard src/common/*.c)

build/board_a/firmware.elf: src/board_a/main.c $(COMMON_SRC)
	@mkdir -p build/board_a
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

build/board_a/firmware.bin: build/board_a/firmware.elf
	$(OBJCOPY) -O binary $< $@

board_a: build/board_a/firmware.bin

build/board_b/firmware.elf: src/board_b/main.c $(COMMON_SRC)
	@mkdir -p build/board_b
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

build/board_b/firmware.bin: build/board_b/firmware.elf
	$(OBJCOPY) -O binary $< $@

board_b: build/board_b/firmware.bin

flash_a: board_a
	st-flash --serial 066CFF555567894967085254 write build/board_a/firmware.bin 0x8000000

flash_b: board_b
	st-flash --serial 0666FF555567894967085233 write build/board_b/firmware.bin 0x8000000

clean:
	rm -fr build/

.PHONY: board_a board_b flash_a flash_b clean