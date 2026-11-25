CC := $(shell xcrun --find clang)
LD := $(shell xcrun --find ld)
PYTHON ?= python3
BUILD_DIR ?= .build

CPPFLAGS := -I. -I../灰度循迹 -I../超声波避障
CFLAGS := -target armv7m-apple-ios -std=c11 -Os -g -ffreestanding \
          -fno-pic -fno-pie -fno-stack-protector -fno-common \
          -Wall -Wextra -Werror -MMD -MP
LDFLAGS := -arch armv7m -static -e _Reset_Handler -u _vector_table -dead_strip -no_uuid \
           -segalign 0x20 -segaddr __VECTORS 0x08000000 -segaddr __TEXT 0x08000400 \
           -segaddr __DATA 0x20000000

VPATH := ../灰度循迹:../超声波避障
OBJECTS := $(addprefix $(BUILD_DIR)/,startup.o sensor_test.o line_follow.o r08_avoidance.o car_control.o)

all: sensor_test.hex

sensor_test.hex: $(BUILD_DIR)/sensor_test.macho macho_to_hex.py
	$(PYTHON) macho_to_hex.py $< $@

$(BUILD_DIR)/sensor_test.macho: $(OBJECTS) Makefile
	$(LD) $(LDFLAGS) -o $@ $(OBJECTS)

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

-include $(OBJECTS:.o=.d)

.PHONY: all
