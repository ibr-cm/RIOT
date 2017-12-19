CPU_TINY=attiny84
CLOCK_TINY=1000000UL

CPP=avr-gcc
OBJCOPY=avr-objcopy
STRIP=avr-strip
SIZE=avr-size
AVRDUDE=avrdude

NAME_TINY=tiny

PROGRAMMER_TYPE_TINY=avrispmkII
PROGRAMMER_PORT_TINY=usb

# use more debug-flags when compiling
DEBUG = 0

EMPTY=

SOURCES_TINY= \
              i2c-master.c \
              i2c-slave.c \
              tiny.c \
              sw_uart.c \
              ${EMPTY}

HEADERS_TINY= \
              i2c-master.h \
              i2c-slave.h \
              usi.h \
              sw_uart.h \
              ../include/idealvolting_config.h \
              ../include/idealvolting_frame.h \
              ${EMPTY}

OBJECTS_TINY=$(shell echo "${SOURCES_TINY}" | sed -e 's/\.c/\.o/g')

OPTIMIZE=-Os
REMOVEDEADCODE=-ffunction-sections -Wl,-gc-sections
# check settings if failures are still detectable
#REMOVEDEADCODE=-fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields

OPTIMIZE_TINY=${OPTIMIZE}
REMOVEDEADCODE_TINY=${REMOVEDEADCODE}
CCFLAGS_TINY=${OPTIMIZE_TINY} -mmcu=${CPU_TINY} ${REMOVEDEADCODE_TINY} -ggdb -Isrc/ -std=c99 -DF_CPU=${CLOCK_TINY} -Wall
LIBS_TINY=

all:${NAME_TINY}
	${SIZE} ${NAME_TINY}

# create config.mk (if it does not exist yet)
$(CURDIR)/config.mk:
	@cp config.mk.template config.mk
	@echo "===================================================="
	@echo "created file $@"
	@echo "there, then run 'make' again"
	@echo "===================================================="
	@exit 1

# include config file
-include $(CURDIR)/config.mk

# include more debug flags, if $(DEBUG) is 1
ifeq ($(DEBUG),1)
	CCFLAGS_TINY += -Wall -W -Wchar-subscripts -Wmissing-prototypes
	CCFLAGS_TINY += -Wmissing-declarations -Wredundant-decls
	CCFLAGS_TINY += -Wstrict-prototypes -Wshadow -Wbad-function-cast
	CCFLAGS_TINY += -Winline -Wpointer-arith -Wsign-compare
	CCFLAGS_TINY += -Wunreachable-code -Wdisabled-optimization
	CCFLAGS_TINY += -Wcast-align -Wwrite-strings -Wnested-externs -Wundef
	CCFLAGS_TINY += -Wa,-adhlns=$(basename $@).lst
	CCFLAGS_TINY += -DCONFIG_DEBUG=1
endif

${NAME_TINY}:${OBJECTS_TINY}
	${CPP} ${CCFLAGS_TINY} ${LIBS_TINY} ${OBJECTS_TINY} -o $@
	${OBJCOPY} -O ihex $@ $@.hex

clean:
	rm -f ${OBJECTS_TINY}
	rm -f ${NAME_TINY}.hex
	rm -f ${NAME_TINY}

upload-tiny:${NAME_TINY}
	${AVRDUDE} -c ${PROGRAMMER_TYPE_TINY} -P ${PROGRAMMER_PORT_TINY} -p ${CPU_TINY} -U flash:w:${NAME_TINY}.hex:i

fuses-tiny:
#todo

upload:upload-tiny

${OBJECTS_TINY}:%.o:%.c ${HEADERS_TINY}
	${CPP} ${CCFLAGS_TINY} $< -c -o $@
