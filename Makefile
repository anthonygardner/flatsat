CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS += -Wall -O2 -g
CFLAGS += -Iinclude -Iinclude/cmsis -Isrc/common

LDFLAGS = -Tstm32f767.ld -nostartfiles

COMMON_SRC = $(wildcard src/common/*.c)

build/obc/firmware.elf: src/obc/main.c $(COMMON_SRC)
	@mkdir -p build/obc
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

build/obc/firmware.bin: build/obc/firmware.elf
	$(OBJCOPY) -O binary $< $@

obc: build/obc/firmware.bin

build/adcs/firmware.elf: src/adcs/main.c $(COMMON_SRC)
	@mkdir -p build/adcs
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

build/adcs/firmware.bin: build/adcs/firmware.elf
	$(OBJCOPY) -O binary $< $@

adcs: build/adcs/firmware.bin

build/eps/firmware.elf: src/eps/main.c $(COMMON_SRC)
	@mkdir -p build/eps
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

build/eps/firmware.bin: build/eps/firmware.elf
	$(OBJCOPY) -O binary $< $@

eps: build/eps/firmware.bin

flash_obc: obc
	st-flash --serial 066EFF485682884967134955 write build/obc/firmware.bin 0x8000000

flash_adcs: adcs
	st-flash --serial 0666FF555567894967085233 write build/adcs/firmware.bin 0x8000000

flash_eps: eps
	st-flash --serial 066CFF555567894967085254 write build/eps/firmware.bin 0x8000000

clean:
	rm -fr build/

can:
	uv run cantools generate_c_source can_bus.dbc -o src/common/

.PHONY: obc adcs eps flash_obc flash_adcs flash_eps clean can