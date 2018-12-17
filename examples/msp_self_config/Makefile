APPLICATION = msp_self_config
BOARD ?= telosb
RIOTBASE ?= $(CURDIR)/../..

USEMODULE += gnrc
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

CFLAGS += -DDEBUG_ASSERT_VERBOSE
CFLAGS += -DGNRC_PKTBUF_SIZE=512

DEVELHELP ?= 1

include $(RIOTBASE)/Makefile.include
