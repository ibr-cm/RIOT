#include <avr/io.h>
#include <util/delay.h>
#include "drv/sw_uart.h"
#include "drv/i2c-master.h"

#define MEGA_ADDR 0x2d

int main(void)
{
	char i2c_buffer[3] = {MEGA_ADDR << 1, 0, 'a'};
	sw_uart_init();
	while (1) {
		_delay_ms(1000);
		i2c_master_transmit(i2c_buffer, sizeof(i2c_buffer));
		printf("Data sent: %c\n", i2c_buffer[2]);
		if (++i2c_buffer[2] > 'z')
			i2c_buffer[2] = 'a';
	}
}
