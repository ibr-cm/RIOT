#include <stdio.h>
#include "periph/i2c_slave.h"

#define MEGA_ADDR 0x2d

void rcb(uint8_t n_received, uint8_t *data_received)
{
	printf("Data received: ");
	for (uint8_t i = 0; i < n_received; ++i)
		printf("%c ", data_received[i]);
	puts("");
}

uint8_t tcb(uint8_t *txbuffer)
{
	puts("Data requested");
	sprintf((char *) txbuffer, "ok");
	return 3;
}

int main(void)
{
	puts("Test for the I2C-slave driver");

	i2c_init_slave(MEGA_ADDR, rcb, tcb);

	while (1) {
		__builtin_avr_delay_cycles(32000000);
		puts("ping");
	}

	return 0;
}
