#include <avr/io.h>
#include <util/delay.h>
#include "drv/sw_uart.h"
#include "drv/usi_i2c_master.h"

#define MEGA_ADDR 0x2d

int main(void)
{
	uint8_t txbuffer[3] = {'a', 'b', 'c'};
	uint8_t rxbuffer[8];
	uint8_t success;
	sw_uart_init();
	i2c_init_master();
	while (1) {
		_delay_ms(1000);
		success = i2c_write_bytes(MEGA_ADDR, txbuffer, sizeof(txbuffer));
		if (success)
			printf("Data sent: %c\n", txbuffer[0]);
		else
			printf("Sending data failed: %u\n", i2c_get_state_info());
		for (uint8_t i = 0; i < sizeof(txbuffer); ++i) {
			if (++txbuffer[i] > 'z')
				txbuffer[i] = 'a';
		}
		_delay_ms(1000);
		success = i2c_read_bytes(MEGA_ADDR, rxbuffer, 3);
		if (success)
			printf("Data read: %s\n", rxbuffer);
		else
			printf("Reading data failed: %u\n", i2c_get_state_info());
	}
}
