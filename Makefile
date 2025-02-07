PROJECT = usbmidi
CFILES = usbmidi.c
DEVICE = stm32f103c8t6
OPENCM3_DIR = libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
