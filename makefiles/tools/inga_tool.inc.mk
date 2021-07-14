FLASHER = $(RIOTBASE)/dist/tools/inga_tool/inga_tool

AVRDUDE_PROGRAMMER = avr109
# Using the FFLAGS here to append the port for the inga_tool and avrdude
FLASHFILE ?= $(HEXFILE)
FFLAGS = -d $(PORT) -r && avrdude -b 230400 -u
FLASHDEPS += $(RIOTBASE)/dist/tools/inga_tool/inga_tool