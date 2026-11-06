# External Make project: also usable with STM32CubeIDE Arm GNU GCC on PATH.
CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size
HOST_CC ?= gcc
BUILD := build
TARGET := $(BUILD)/rcrc_h723
include firmware/platform/platform.mk
SOURCES := firmware/src/main.c firmware/src/app.c firmware/src/control.c $(PLATFORM_SOURCES)
OBJECTS := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(SOURCES))))
CPU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CPPFLAGS := $(addprefix -I,firmware/include $(PLATFORM_INCLUDES)) $(addprefix -D,$(PLATFORM_DEFINES))
CFLAGS := $(CPU) -std=c11 -O2 -g3 -ffunction-sections -fdata-sections -fno-common -Wall -Wextra
ASFLAGS := $(CPU) -g3 -x assembler-with-cpp
LDFLAGS := $(CPU) -Tfirmware/platform/STM32H723ZGTX_FLASH.ld --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections,-Map=$(TARGET).map -Wl,-u,_printf_float
LDLIBS := -lm
.PHONY: all clean host host-test gui-lib
all: $(TARGET).hex $(TARGET).bin
$(TARGET).elf: $(OBJECTS) firmware/platform/STM32H723ZGTX_FLASH.ld
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@
	$(SIZE) $@
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(ASFLAGS) -c $< -o $@
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
host-test:
	@mkdir -p $(BUILD)
	$(HOST_CC) -std=c11 -O2 -Wall -Wextra -Werror -Ifirmware/include firmware/src/control.c tests/test_control.c -lm -o $(BUILD)/test_control
	$(BUILD)/test_control
	$(HOST_CC) -std=c11 -O2 -Wall -Wextra -Werror -Ifirmware/include firmware/src/control.c firmware/src/app.c tests/test_app.c -lm -o $(BUILD)/test_app
	$(BUILD)/test_app
gui-lib:
	@mkdir -p $(BUILD)
	$(HOST_CC) -std=c11 -O2 -shared -fPIC -Ifirmware/include desktop/sim_bridge.c firmware/src/control.c -lm -o $(BUILD)/libsimulation.so
host: host-test gui-lib
clean:
	rm -rf $(BUILD)
-include $(OBJECTS:.o=.d)
