include ../Makefile.net_common

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../..

# Include packages that pull up and auto-init the link layer.
USEMODULE += netdev_default
USEMODULE += auto_init_gnrc_netif
# Include dmts_mac (this will include periph_rtc)
USEMODULE += gnrc_dmts_mac
USEMODULE += gnrc_dmts_mac_skew_detection # Pseudo module to enable the skew detection feature
FEATURES_REQUIRED += periph_rtc

# Use only one netif of the at86rf215 radio
USEMODULE += at86rf215
USEMODULE += at86rf215_24ghz

# Configure the radio
SPI ?= SPI_DEV(0)
PIN_CS ?= GPIO_PIN(PORT_B, 6)
PIN_IRQ ?= GPIO_PIN(PORT_A, 9)
PIN_RST ?= GPIO_PIN(PORT_C, 7)

ifneq (, $(SPI))
  CFLAGS += '-DAT86RF215_PARAM_SPI=$(SPI)'
endif
ifneq (, $(PIN_CS))
  CFLAGS += '-DAT86RF215_PARAM_CS=$(PIN_CS)'
endif
ifneq (, $(PIN_IRQ))
  CFLAGS += '-DAT86RF215_PARAM_INT=$(PIN_IRQ)'
endif
ifneq (, $(PIN_RST))
  CFLAGS += '-DAT86RF215_PARAM_RESET=$(PIN_RST)'
endif

# Sets the interval for calculating the clock drift
# DMTS_INTERVAL=1 -> 1s
# DMTS_INTERVAL=32 -> 32s (or leave it out)
# DMTS_INTERVAL=64 -> 64s
ifneq (,$(DMTS_INTERVAL))
  USEMODULE += gnrc_dmts_mac_sync_interval_$(DMTS_INTERVAL)s
endif

# Use this to select the master node (DMTS_MASTER=something)
ifneq (,$(DMTS_MASTER))
  USEMODULE += gnrc_dmts_mac_skew_detection_master
endif

DEVELHELP ?= 1
QUIET ?= 1

EXTERNAL_BOARD_DIRS += $(RIOTBASE)/tests/build_system/external_board_dirs/esp-ci-boards

include $(RIOTBASE)/Makefile.include

# Set a custom channel if needed
include $(RIOTMAKE)/default-radio-settings.inc.mk
