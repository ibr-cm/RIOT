FLASHER = $(RIOTBASE)/dist/tools/inga_tool/inga_tool
# Using the FFLAGS here to append the port for the inga_tool and avrdude
FLASHFILE ?= $(HEXFILE)
export FFLAGS = -d $(PORT) -r && avrdude -c avr109 -p m1284p -P $(PORT) -b 230400 -u -U flash:w:$(FLASHFILE)
FLASHDEPS += $(RIOTBASE)/dist/tools/inga_tool/inga_tool
