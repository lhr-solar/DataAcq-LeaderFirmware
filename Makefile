# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# Build Directory
BUILD_DIR := build

# Flags
CFLAGS = -mcpu=cortex-m4 -mthumb -O0 -g -Wall -nostdlib -ffreestanding
LDFLAGS = -T config/linker.ld -nostartfiles
INCLUDES = -I./CMSIS/Include -I./CMSIS/Device/ST

# Files
SRCS 	:= $(wildcard src/*.c)
OBJS 	:= $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SRCS))
OUT_ELF := $(BUILD_DIR)/firmware.elf
OUT_BIN := $(BUILD_DIR)/firmware.bin

# Phony Targets
.PHONY: all bin clean

# Build targets
all: $(OUT_ELF)

# Link
$(OUT_ELF): $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@ $(LDFLAGS)

# Compile
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

bin: $(OUT_BIN)

$(OUT_BIN): $(OUT_ELF)
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

