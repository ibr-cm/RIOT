# name of your application
APPLICATION = sram_undervolting

BOARD = reaper

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += bme280_i2c
USEMODULE += fmt

#Initialisiere Link-Layer und Netzwerkinterface
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

FEATURES_REQUIRED = periph_adc

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

DEVELHELP ?= 1

include $(RIOTBASE)/Makefile.include
