OPENCM3_DIR = libopencm3
PROJECT = usbmidi
CFILES = usbmidi.c
#DEVICE = stm32f103c8t6
LDSCRIPT = stm32f103c8t6.ld
OPENCM3_LIB = opencm3_stm32f1
OPENCM3_DEFS = -DSTM32F1
ARCH_FLAGS = -mcpu=cortex-m3 -mthumb -msoft-float
OPT = -O2

#include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
#include $(OPENCM3_DIR)/mk/genlink-rules.mk
