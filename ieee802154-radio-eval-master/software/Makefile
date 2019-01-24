APPLICATION=software
BOARD ?= inga_blue
RIOTBASE ?= $(CURDIR)/../../

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += saul_default
USEMODULE += gnrc
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
#USEMODULE += gnrc_txtsnd
#USEMODULE += gnrc_pktdump
USEMODULE += periph_i2c

NODE_ID ?= 0
CFLAGS += -DNODE_ID=$(NODE_ID)
CFLAGS += -DDEBUG_ASSERT_VERBOSE
CFLAGS += -DGNRC_PKTBUF_SIZE=512
CFLAGS += -DSHELL_NO_ECHO -DSHELL_NO_PROMPT

BAUD := 9600

CFLAGS += -DUART_STDIO_BAUDRATE=9600

DEVELHELP ?= 1
# No debug information
#CFLAGS += -DNDEBUG

include $(RIOTBASE)/Makefile.include
