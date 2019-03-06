include ../Makefile.tests_common

BOARD_INSUFFICIENT_MEMORY := arduino-duemilanove arduino-uno

FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_gpio_irq

USEMODULE += atmega_pcint

include $(RIOTBASE)/Makefile.include

CFLAGS += "-DTHREAD_STACKSIZE_DEFAULT=256"
