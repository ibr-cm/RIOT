#include <avr/io.h>
#include <util/delay.h>
#include "sw_uart.h"

int main(void)
{
#ifdef BOARD_REAPER
	DDRA &=~(1<<PA2);   // VBAT_OK input test
	DDRA |= (1<<PA1);   // Enable i2c level shifter
	DDRA |= (1<<PA3);   // VOUT_EN buck enable test
	PORTA |= (1<<PA1);  // Enable i2c level shifter
	PORTA |= (1<<PA3);  // Enable buck
#endif
	sw_uart_init();

	while (1) {
		_delay_ms(1000);
		puts("idle");
	}
}
