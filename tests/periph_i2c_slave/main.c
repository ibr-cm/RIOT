#include <stdio.h>
#include "periph/i2c_slave.h"

#define MEGA_ADDR 0x2d

int main(void)
{
	puts("Test for the I2C-slave driver");

	uint8_t c = 0;

	i2c_init_slave(MEGA_ADDR);

	while (1) {
		if (rxbuffer[0] != c) {
			c = rxbuffer[0];
			printf("Data received: %c\n", c);
		}
	}

	return 0;
}
