#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU               (1000000UL) // 1.0 MHz std clock rate of Attiny84
#endif 


/*Define UART Pins*/
#define SW_UART_PORT        PORTA
#define SW_UART_DDR         DDRA
#define SW_UART_TX          PA0
//#define SW_UART_RX          PB1

#define SW_UART_BAUDRATE   (19200)

#define TXDELAY             (int)(((F_CPU/SW_UART_BAUDRATE)-7 +1.5)/3)
#define RXDELAY             (int)(((F_CPU/SW_UART_BAUDRATE)-5 +1.5)/3)
#define RXDELAY2            (int)((RXDELAY*1.5)-2.5)
#define RXROUNDED           (((F_CPU/SW_UART_BAUDRATE)-5 +2)/3)

#if RXROUNDED > 127
# error Low baud rates unsupported - use higher UART_BAUDRATE
#endif

void sw_uart_init(void);
#ifdef SW_UART_RX
char sw_uart_getc(void);
#endif
void sw_uart_putc(char c);
void sw_uart_puts(const char *s);
