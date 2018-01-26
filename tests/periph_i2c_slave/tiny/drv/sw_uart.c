#include "sw_uart.h"

static int sw_uart_putchar(char c, FILE *stream);
static FILE sw_uart_stdout = FDEV_SETUP_STREAM(sw_uart_putchar, NULL, _FDEV_SETUP_WRITE);

void sw_uart_init(void){
    SW_UART_PORT |= 1 << SW_UART_TX;
    SW_UART_DDR |= 1 << SW_UART_TX;
#ifdef SW_UART_RX
    SW_UART_PORT &= ~(1 << SW_UART_RX);
    SW_UART_DDR &= ~(1 << SW_UART_RX);
#endif
    /*redirect stdout to sw uart*/
	stdout = &sw_uart_stdout;
}

#ifdef SW_UART_RX
char sw_uart_getc(void){
    char c;
    uint8_t sreg;

    sreg = SREG;
    cli();
    SW_UART_PORT &= ~(1 << SW_UART_RX);
    SW_UART_DDR &= ~(1 << SW_UART_RX);
    __asm volatile(
        " ldi r18, %[rxdelay2] \n\t" // 1.5 bit delay
        " ldi %0, 0x80 \n\t" // bit shift counter
        "WaitStart: \n\t"
        " sbic %[uart_port]-2, %[uart_pin] \n\t" // wait for start edge
        " rjmp WaitStart \n\t"
        "RxBit: \n\t"
        // 6 cycle loop + delay - total = 5 + 3*r22
        // delay (3 cycle * r18) -1 and clear carry with subi
        " subi r18, 1 \n\t"
        " brne RxBit \n\t"
        " ldi r18, %[rxdelay] \n\t"
        " sbic %[uart_port]-2, %[uart_pin] \n\t" // check UART PIN
        " sec \n\t"
        " ror %0 \n\t"
        " brcc RxBit \n\t"
        "StopBit: \n\t"
        " dec r18 \n\t"
        " brne StopBit \n\t"
        : "=r" (c)
        : [uart_port] "I" (_SFR_IO_ADDR(SW_UART_PORT)),
        [uart_pin] "I" (SW_UART_RX),
        [rxdelay] "I" (RXDELAY),
        [rxdelay2] "I" (RXDELAY2)
        : "r0","r18","r19"
    );
    SREG = sreg;
    return c;
}
#endif /* SW_UART_RX */

void sw_uart_putc(char c){
    uint8_t sreg;

    sreg = SREG;
    cli();
    SW_UART_PORT |= 1 << SW_UART_TX;
    SW_UART_DDR |= 1 << SW_UART_TX;
    __asm volatile(
        " cbi %[uart_port], %[uart_pin] \n\t" // start bit
        " in r0, %[uart_port] \n\t"
        " ldi r30, 3 \n\t" // stop bit + idle state
        " ldi r28, %[txdelay] \n\t"
        "TxLoop: \n\t"
        // 8 cycle loop + delay - total = 7 + 3*r22
        " mov r29, r28 \n\t"
        "TxDelay: \n\t"
        // delay (3 cycle * delayCount) - 1
        " dec r29 \n\t"
        " brne TxDelay \n\t"
        " bst %[ch], 0 \n\t"
        " bld r0, %[uart_pin] \n\t"
        " lsr r30 \n\t"
        " ror %[ch] \n\t"
        " out %[uart_port], r0 \n\t"
        " brne TxLoop \n\t"
        :
        : [uart_port] "I" (_SFR_IO_ADDR(SW_UART_PORT)),
        [uart_pin] "I" (SW_UART_TX),
        [txdelay] "I" (TXDELAY),
        [ch] "r" (c)
        : "r0","r28","r29","r30"
    );
    SREG = sreg;
}

static int sw_uart_putchar(char c, FILE *stream) {
	if (c == '\n')
		sw_uart_putchar('\r', stream);

	sw_uart_putc(c);
	return 0;
}


void sw_uart_puts(const char *s){
         while (*s) sw_uart_putc(*(s++));
}
