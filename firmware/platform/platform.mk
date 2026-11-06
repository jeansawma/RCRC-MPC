PLATFORM_INCLUDES := firmware/platform vendor/hal/Inc vendor/device/Include vendor/cmsis/Include vendor/usb/Core/Inc vendor/usb/Class/CDC/Inc
PLATFORM_DEFINES := STM32H723xx USE_HAL_DRIVER HSE_VALUE=16000000U
PLATFORM_SOURCES := $(wildcard firmware/platform/*.c) $(wildcard vendor/hal/Src/*.c) $(filter-out %_template.c,$(wildcard vendor/usb/Core/Src/*.c) $(wildcard vendor/usb/Class/CDC/Src/*.c)) vendor/device/system_stm32h7xx.c vendor/device/startup_stm32h723xx.s
PLATFORM_LDSCRIPT := firmware/platform/STM32H723ZGTX_FLASH.ld
